// Offline bag runner: reads an rosbag2 bag, runs madodom odometry, writes TUM.
// Usage: madodom_offline <bag_dir> <output_tum> [intensity_min]
// Output format: "timestamp tx ty tz qx qy qz qw"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Geometry>

#include "madodom_core/odometry.hpp"
#include "madodom_core/types.hpp"

static madodom::PointCloud parseCloud(
    const sensor_msgs::msg::PointCloud2& msg, double intensity_min)
{
    madodom::PointCloud pts;
    pts.reserve(msg.width * msg.height);

    int off_x = -1, off_y = -1, off_z = -1, off_i = -1;
    for (const auto& f : msg.fields) {
        if      (f.name == "x")         off_x = static_cast<int>(f.offset);
        else if (f.name == "y")         off_y = static_cast<int>(f.offset);
        else if (f.name == "z")         off_z = static_cast<int>(f.offset);
        else if (f.name == "intensity") off_i = static_cast<int>(f.offset);
    }
    if (off_x < 0 || off_y < 0 || off_z < 0) return pts;

    const bool filter_i = (intensity_min > 0.0) && (off_i >= 0);
    const uint8_t* data = msg.data.data();

    for (uint32_t row = 0; row < msg.height; ++row) {
        for (uint32_t col = 0; col < msg.width; ++col) {
            const uint8_t* p = data + row * msg.row_step + col * msg.point_step;
            float x, y, z;
            std::memcpy(&x, p + off_x, 4);
            std::memcpy(&y, p + off_y, 4);
            std::memcpy(&z, p + off_z, 4);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
            if (filter_i) {
                float iv; std::memcpy(&iv, p + off_i, 4);
                if (iv < static_cast<float>(intensity_min)) continue;
            }
            pts.emplace_back(static_cast<double>(x),
                             static_cast<double>(y),
                             static_cast<double>(z));
        }
    }
    return pts;
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::fprintf(stderr,
            "Usage: madodom_offline <bag_dir> <output_tum> [intensity_min]\n"
            "  bag_dir      : path to the ros2 bag directory\n"
            "  output_tum   : path for TUM trajectory output (e.g. estimate.txt)\n"
            "  intensity_min: drop points below this intensity (default 1100)\n");
        return 1;
    }

    const std::string bag_path     = argv[1];
    const std::string out_path     = argv[2];
    const double      intensity_min = (argc > 3) ? std::stod(argv[3]) : 1100.0;

    // ── Odometry config (mirrors madodom.yaml) ────────────────────────────
    madodom::OdometryConfig cfg;
    cfg.min_range           = 1.0;
    cfg.max_range           = 45.0;
    cfg.b_max               = 0.6;
    cfg.b_min               = 0.1;
    cfg.b_ratio             = 0.02;
    cfg.num_keyframes       = 8;
    cfg.max_iterations      = 15;
    cfg.delta_chi_eps       = 1e-6;
    cfg.rho_ker             = 0.1;
    cfg.p_th                = 0.8;
    cfg.min_inlier_ratio    = 0.4;
    cfg.max_chi_health      = 1.0;
    cfg.max_chi_keyframe    = 0.15;
    cfg.min_correspondences = 50;
    cfg.sensor_hz           = 10.0;
    cfg.frame_window        = 10;

    madodom::Odometry odometry(cfg);

    // ── Open bag ──────────────────────────────────────────────────────────
    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri        = bag_path;
    storage_opts.storage_id = "mcap";   // auto-detected if wrong

    rosbag2_cpp::ConverterOptions converter_opts;
    converter_opts.input_serialization_format  = "cdr";
    converter_opts.output_serialization_format = "cdr";

    rosbag2_cpp::readers::SequentialReader reader;
    try {
        reader.open(storage_opts, converter_opts);
    } catch (const std::exception& e) {
        // Try sqlite3 format
        storage_opts.storage_id = "sqlite3";
        try {
            reader.open(storage_opts, converter_opts);
        } catch (const std::exception& e2) {
            // Let rosbag2 auto-detect
            storage_opts.storage_id = "";
            reader.open(storage_opts, converter_opts);
        }
    }

    // Find the /ouster/points topic
    const std::string lidar_topic = "/ouster/points";
    rosbag2_storage::StorageFilter filter;
    filter.topics.push_back(lidar_topic);
    reader.set_filter(filter);

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

    // ── Output file ───────────────────────────────────────────────────────
    std::ofstream out(out_path);
    if (!out) {
        std::fprintf(stderr, "Cannot open output file: %s\n", out_path.c_str());
        return 1;
    }

    std::fprintf(stderr, "Processing bag: %s\n", bag_path.c_str());
    std::fprintf(stderr, "intensity_min = %.0f\n", intensity_min);

    int scan_count = 0;
    while (reader.has_next()) {
        auto msg_bag = reader.read_next();
        if (msg_bag->topic_name != lidar_topic) continue;

        // Deserialize
        rclcpp::SerializedMessage serialized(*msg_bag->serialized_data);
        sensor_msgs::msg::PointCloud2 cloud_msg;
        serializer.deserialize_message(&serialized, &cloud_msg);

        // Timestamp in seconds
        const double ts = cloud_msg.header.stamp.sec
                        + cloud_msg.header.stamp.nanosec * 1e-9;

        // Parse + filter points
        const madodom::PointCloud pts = parseCloud(cloud_msg, intensity_min);

        // Register scan
        const madodom::OdometryResult res = odometry.registerScan(pts);

        // Extract pose
        const madodom::Pose& T = res.pose;
        const Eigen::Quaterniond q(T.linear());
        const Eigen::Vector3d&   t = T.translation();

        // Write TUM line
        out << std::fixed << std::setprecision(9)
            << ts                << " "
            << t.x()             << " " << t.y() << " " << t.z() << " "
            << q.x()             << " " << q.y() << " " << q.z() << " " << q.w()
            << "\n";

        ++scan_count;
        if (scan_count % 100 == 0)
            std::fprintf(stderr, "  scan %4d  pos=(%.2f, %.2f, %.2f)\n",
                         scan_count, t.x(), t.y(), t.z());
    }

    out.close();
    std::fprintf(stderr, "Done. %d scans → %s\n", scan_count, out_path.c_str());
    return 0;
}
