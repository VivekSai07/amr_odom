#pragma once

#include "madodom_core/icp.hpp"
#include "madodom_core/local_map.hpp"
#include "madodom_core/types.hpp"

#include <deque>
#include <vector>

namespace madodom {

struct OdometryConfig {
    // ── Sensor / preprocessing ────────────────────────────────────────────
    double min_range = 1.0;      // [m] drop points closer than this
    double max_range = 45.0;     // [m] drop points farther than this

    // ── MAD-tree / map ────────────────────────────────────────────────────
    double b_max         = 0.6;  // max leaf extent; also base search radius [m]
    double b_min         = 0.1;  // thin-plane threshold for normal inheritance [m]
    double b_ratio       = 0.02; // search radius grows b_ratio·‖source_point‖
    int    num_keyframes = 8;    // sliding-window keyframe count

    // ── ICP ───────────────────────────────────────────────────────────────
    int    max_iterations      = 15;
    double delta_chi_eps       = 1e-6;
    double rho_ker             = 0.1;   // Huber threshold² (sqrt taken in solver)
    double p_th                = 0.8;   // inlier ratio below which new keyframe added
    double min_inlier_ratio    = 0.4;   // absolute floor: below this → freeze pose
    double max_chi_health      = 1.0;   // loose chi gate: above this → freeze pose [m]
    double max_chi_keyframe    = 1.0;   // only add keyframe when mean chi < this [m]
    std::size_t min_correspondences = 50;

    // ── Velocity estimator / frame buffer ─────────────────────────────────
    double sensor_hz   = 10.0;  // LiDAR frequency (for dt in velocity predictor)
    int    frame_window = 10;   // frames kept for best-H keyframe selection
};

struct OdometryResult {
    Pose        pose                = Pose::Identity();
    bool        is_first_scan       = false;
    bool        icp_healthy         = false;
    bool        icp_converged       = false;
    int         icp_iterations      = 0;
    std::size_t num_correspondences = 0;
    double      inlier_ratio        = 0.0;
    double      mean_chi            = 0.0;
    bool        new_keyframe        = false;
    std::size_t num_input_points    = 0;
    std::size_t num_source_leaves   = 0;
};

// Huber-robust constant-velocity estimator over a sliding pose window.
// Mirrors the VelEstimator in reference MAD-ICP.
class VelEstimator {
public:
    explicit VelEstimator(double sensor_hz)
        : ts_(1.0 / sensor_hz) {}

    void push(const Pose& T) {
        window_.push_back(T);
        if ((int)window_.size() > kSmoothing) window_.pop_front();
    }

    // Run one GN round; must call push() first.
    void oneRound() {
        if ((int)window_.size() < 2) return;
        Mat6 H = Mat6::Zero();
        Vec6 b = Vec6::Zero();
        const Pose& T_now = window_.back();
        const int n = static_cast<int>(window_.size());
        for (int i = 0; i < n - 1; ++i) {
            const double dt     = static_cast<double>(n - 1 - i) * ts_;
            const double weight = static_cast<double>(i + 1) / static_cast<double>(n);
            const Pose   dT     = window_[i].inverse() * T_now;
            Vec6 e;
            e.head<3>() = dt * X_.head<3>() - dT.translation();
            e.tail<3>() = dt * X_.tail<3>() - logMapSO3(dT.linear());
            const double chi = e.norm();
            const double scale = (chi > kEThresh) ? kEThresh / chi : 1.0;
            const Mat6 J = Mat6::Identity() * dt;
            H += scale * weight * J.transpose() * J;
            b += scale * weight * J.transpose() * e;
        }
        X_prev_ = X_;
        const Vec6 dx = H.ldlt().solve(-b);
        X_ += dx;
    }

    // Trapezoidal SE(3) prediction: T_curr * exp(0.5*(v_curr+v_prev)*dt).
    Pose predict(const Pose& T_curr) const {
        const double dt = ts_;
        const Vec6 xi = 0.5 * (X_ + X_prev_) * dt;
        Pose dX;
        dX.linear()      = expMapSO3(xi.tail<3>());
        dX.translation() = xi.head<3>();
        return T_curr * dX;
    }

    bool hasEnoughData() const { return (int)window_.size() >= 3; }

    void reset() {
        window_.clear();
        X_.setZero();
        X_prev_.setZero();
    }

private:
    static constexpr double kEThresh   = 0.3162;
    static constexpr int    kSmoothing = 10;

    double ts_;
    Vec6   X_      = Vec6::Zero();   // current velocity estimate [v; omega]
    Vec6   X_prev_ = Vec6::Zero();   // velocity before last update (trapezoidal)
    std::deque<Pose> window_;
};

class Odometry {
public:
    Odometry();
    explicit Odometry(const OdometryConfig& cfg);

    void reset();

    OdometryResult registerScan(const PointCloud& scan_sensor);

    const Pose& currentPose() const { return T_curr_; }

private:
    Pose predictPose() const;

    struct FrameInfo {
        Pose              pose;
        Mat6              H;
        std::vector<Vec3> world_pts;
    };

    OdometryConfig         cfg_;
    LocalMap               map_;
    VelEstimator           vel_;

    Pose T_curr_ = Pose::Identity();
    int  n_scans_ = 0;

    std::deque<FrameInfo> frame_buffer_;
};

}  // namespace madodom
