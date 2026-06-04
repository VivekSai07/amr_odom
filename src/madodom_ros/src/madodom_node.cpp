#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <deque>
#include <mutex>
#include <thread>

#include "madodom_core/odometry.hpp"
#include "madodom_core/types.hpp"

class MadodomNode : public rclcpp::Node {
public:
    explicit MadodomNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
        : Node("madodom", opts) {
        // ── Declare + load parameters ─────────────────────────────────────
        declare_parameter("lidar_topic",    "/ouster/points");
        declare_parameter("odom_frame",     "odom");
        declare_parameter("base_frame",     "base_link");
        declare_parameter("publish_tf",     true);
        declare_parameter("publish_odom",   true);
        declare_parameter("publish_path",   true);
        declare_parameter("max_path_poses", 10000);

        declare_parameter("min_range",       1.0);
        declare_parameter("max_range",      45.0);
        declare_parameter("intensity_min",   0.0);

        declare_parameter("b_max",           0.6);
        declare_parameter("b_min",           0.1);
        declare_parameter("b_ratio",         0.02);
        declare_parameter("num_keyframes",   8);
        declare_parameter("p_th",            0.8);

        declare_parameter("icp_max_iterations",      15);
        declare_parameter("icp_delta_chi_eps",        1e-6);
        declare_parameter("icp_rho_ker",              0.1);
        declare_parameter("icp_min_correspondences",  50);
        declare_parameter("icp_min_inlier_ratio",     0.4);
        declare_parameter("icp_max_chi_health",       1.0);
        declare_parameter("icp_max_chi_keyframe",     1.0);
        declare_parameter("sensor_hz",               10.0);
        declare_parameter("frame_window",            10);
        declare_parameter("source_voxel_size",        0.3);

        lidar_topic_   = get_parameter("lidar_topic").as_string();
        odom_frame_    = get_parameter("odom_frame").as_string();
        base_frame_    = get_parameter("base_frame").as_string();
        publish_tf_    = get_parameter("publish_tf").as_bool();
        publish_odom_  = get_parameter("publish_odom").as_bool();
        publish_path_  = get_parameter("publish_path").as_bool();
        max_path_poses_= static_cast<std::size_t>(get_parameter("max_path_poses").as_int());
        intensity_min_ = get_parameter("intensity_min").as_double();

        madodom::OdometryConfig cfg;
        cfg.min_range     = get_parameter("min_range").as_double();
        cfg.max_range     = get_parameter("max_range").as_double();
        cfg.b_max         = get_parameter("b_max").as_double();
        cfg.b_min         = get_parameter("b_min").as_double();
        cfg.b_ratio       = get_parameter("b_ratio").as_double();
        cfg.num_keyframes = static_cast<int>(get_parameter("num_keyframes").as_int());
        cfg.p_th          = get_parameter("p_th").as_double();
        cfg.max_iterations= static_cast<int>(get_parameter("icp_max_iterations").as_int());
        cfg.delta_chi_eps = get_parameter("icp_delta_chi_eps").as_double();
        cfg.rho_ker       = get_parameter("icp_rho_ker").as_double();
        cfg.min_correspondences =
            static_cast<std::size_t>(get_parameter("icp_min_correspondences").as_int());
        cfg.min_inlier_ratio    = get_parameter("icp_min_inlier_ratio").as_double();
        cfg.max_chi_health      = get_parameter("icp_max_chi_health").as_double();
        cfg.max_chi_keyframe    = get_parameter("icp_max_chi_keyframe").as_double();
        cfg.sensor_hz           = get_parameter("sensor_hz").as_double();
        cfg.frame_window        = static_cast<int>(get_parameter("frame_window").as_int());
        cfg.source_voxel_size   = get_parameter("source_voxel_size").as_double();

        odometry_ = std::make_unique<madodom::Odometry>(cfg);

        RCLCPP_INFO(get_logger(),
            "madodom: range=[%.1f,%.1f] b_max=%.2f p_th=%.2f "
            "keyframes=%d intensity_min=%.0f",
            cfg.min_range, cfg.max_range, cfg.b_max, cfg.p_th,
            cfg.num_keyframes, intensity_min_);

        // ── Publishers ────────────────────────────────────────────────────
        if (publish_tf_)
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        if (publish_odom_)
            odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 10);
        if (publish_path_)
            path_pub_ = create_publisher<nav_msgs::msg::Path>("path",
                rclcpp::QoS(1));

        // ── Subscriber + async worker thread ─────────────────────────────
        scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_, rclcpp::QoS(10),
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                if (scan_queue_.size() >= kMaxQueue) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "queue full, dropping scan");
                    return;
                }
                scan_queue_.push_back(std::move(msg));
                lock.unlock();
                queue_cv_.notify_one();
            });

        worker_thread_ = std::thread([this] { workerLoop(); });
    }

    ~MadodomNode() override {
        shutdown_.store(true);
        queue_cv_.notify_all();
        if (worker_thread_.joinable()) worker_thread_.join();
    }

private:
    static constexpr std::size_t kMaxQueue = 4;

    void workerLoop() {
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [this] {
                return !scan_queue_.empty() || shutdown_.load();
            });
            if (shutdown_.load() && scan_queue_.empty()) return;
            auto msg = scan_queue_.front();
            scan_queue_.pop_front();
            lock.unlock();
            processScan(msg);
        }
    }

    void processScan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
        // ── Parse PointCloud2 → Vec3 vector ──────────────────────────────
        madodom::PointCloud pts;
        pts.reserve(msg->width * msg->height);

        const uint8_t* data = msg->data.data();
        const uint32_t row_step = msg->row_step;
        const uint32_t point_step = msg->point_step;

        // Find field offsets.
        int off_x = -1, off_y = -1, off_z = -1, off_intensity = -1;
        for (const auto& f : msg->fields) {
            if (f.name == "x")         off_x         = static_cast<int>(f.offset);
            else if (f.name == "y")    off_y         = static_cast<int>(f.offset);
            else if (f.name == "z")    off_z         = static_cast<int>(f.offset);
            else if (f.name == "intensity") off_intensity = static_cast<int>(f.offset);
        }

        if (off_x < 0 || off_y < 0 || off_z < 0) {
            RCLCPP_ERROR(get_logger(), "PointCloud2 missing x/y/z fields");
            return;
        }

        const bool filter_intensity = (intensity_min_ > 0.0) && (off_intensity >= 0);

        for (uint32_t row = 0; row < msg->height; ++row) {
            for (uint32_t col = 0; col < msg->width; ++col) {
                const uint8_t* p = data + row * row_step + col * point_step;

                float x, y, z;
                std::memcpy(&x, p + off_x, sizeof(float));
                std::memcpy(&y, p + off_y, sizeof(float));
                std::memcpy(&z, p + off_z, sizeof(float));

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                    continue;

                if (filter_intensity) {
                    float intensity;
                    std::memcpy(&intensity, p + off_intensity, sizeof(float));
                    if (intensity < static_cast<float>(intensity_min_)) continue;
                }

                pts.emplace_back(static_cast<double>(x),
                                 static_cast<double>(y),
                                 static_cast<double>(z));
            }
        }

        // ── Register ──────────────────────────────────────────────────────
        const madodom::OdometryResult result = odometry_->registerScan(pts);

        // ── Publish ───────────────────────────────────────────────────────
        const rclcpp::Time stamp = msg->header.stamp;
        const madodom::Pose& T   = result.pose;

        const Eigen::Quaterniond q(T.linear());
        const Eigen::Vector3d&   t = T.translation();

        if (publish_tf_ && tf_broadcaster_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp            = stamp;
            tf_msg.header.frame_id         = odom_frame_;
            tf_msg.child_frame_id          = base_frame_;
            tf_msg.transform.translation.x = t.x();
            tf_msg.transform.translation.y = t.y();
            tf_msg.transform.translation.z = t.z();
            tf_msg.transform.rotation.x    = q.x();
            tf_msg.transform.rotation.y    = q.y();
            tf_msg.transform.rotation.z    = q.z();
            tf_msg.transform.rotation.w    = q.w();
            tf_broadcaster_->sendTransform(tf_msg);
        }

        if (publish_odom_ && odom_pub_) {
            nav_msgs::msg::Odometry odom;
            odom.header.stamp            = stamp;
            odom.header.frame_id         = odom_frame_;
            odom.child_frame_id          = base_frame_;
            odom.pose.pose.position.x    = t.x();
            odom.pose.pose.position.y    = t.y();
            odom.pose.pose.position.z    = t.z();
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();
            odom_pub_->publish(odom);
        }

        if (publish_path_ && path_pub_) {
            geometry_msgs::msg::PoseStamped ps;
            ps.header.stamp            = stamp;
            ps.header.frame_id         = odom_frame_;
            ps.pose.position.x         = t.x();
            ps.pose.position.y         = t.y();
            ps.pose.position.z         = t.z();
            ps.pose.orientation.x      = q.x();
            ps.pose.orientation.y      = q.y();
            ps.pose.orientation.z      = q.z();
            ps.pose.orientation.w      = q.w();

            path_.header.stamp    = stamp;
            path_.header.frame_id = odom_frame_;
            path_.poses.push_back(ps);
            if (path_.poses.size() > max_path_poses_)
                path_.poses.erase(path_.poses.begin());
            // Publish at ~2 Hz instead of every scan: with depth-1 QoS the
            // publisher drops any unread message the moment a newer one arrives,
            // but throttling further reduces DDS load for large path messages.
            if (++path_pub_counter_ % 5 == 0)
                path_pub_->publish(path_);
        }
    }

    // ── Members ───────────────────────────────────────────────────────────────
    std::unique_ptr<madodom::Odometry>            odometry_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr      path_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;

    nav_msgs::msg::Path path_;
    std::size_t         path_pub_counter_ = 0;

    std::string lidar_topic_, odom_frame_, base_frame_;
    bool        publish_tf_, publish_odom_, publish_path_;
    std::size_t max_path_poses_;
    double      intensity_min_;

    std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> scan_queue_;
    std::mutex    queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread   worker_thread_;
    std::atomic<bool> shutdown_{false};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MadodomNode>());
    rclcpp::shutdown();
    return 0;
}
