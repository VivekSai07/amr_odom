#include "madodom_core/odometry.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

#include "madodom_core/voxel_grid.hpp"

namespace madodom {

static LocalMapConfig mapCfgFrom(const OdometryConfig& c) {
    LocalMapConfig mc;
    mc.num_keyframes = c.num_keyframes;
    mc.p_th          = c.p_th;
    mc.b_max         = c.b_max;
    mc.b_min         = c.b_min;
    return mc;
}

Odometry::Odometry() : Odometry(OdometryConfig{}) {}

Odometry::Odometry(const OdometryConfig& cfg)
    : cfg_(cfg), map_(mapCfgFrom(cfg)), vel_(cfg.sensor_hz) {
    T_curr_.setIdentity();
}

void Odometry::reset() {
    map_.clear();
    T_curr_.setIdentity();
    vel_.reset();
    frame_buffer_.clear();
    n_scans_ = 0;
}

// Trapezoidal velocity-estimator prediction, with constant-velocity fallback.
Pose Odometry::predictPose() const {
    if (n_scans_ < 2) return T_curr_;
    if (!vel_.hasEnoughData()) return T_curr_;
    const Pose T_pred = vel_.predict(T_curr_);
    if (!T_pred.matrix().allFinite() || T_pred.translation().norm() > 1e4)
        return T_curr_;
    return T_pred;
}

OdometryResult Odometry::registerScan(const PointCloud& scan_sensor) {
    OdometryResult result;
    result.num_input_points = scan_sensor.size();

    // ── Range filter ────────────────────────────────────────────────────────
    const PointCloud filtered = filterByRange(scan_sensor, cfg_.min_range, cfg_.max_range);
    result.num_source_leaves = filtered.size();

    // ── First scan: seed the map, return identity ───────────────────────────
    // (Uses raw filtered cloud for the initial keyframe — no source tree needed.)
    if (n_scans_ == 0) {
        if (!filtered.empty()) {
            std::vector<Vec3> world_pts(filtered.begin(), filtered.end());
            map_.addKeyframe(std::move(world_pts));
        }
        result.pose          = T_curr_;
        result.is_first_scan = true;
        n_scans_ = 1;
        return result;
    }

    // ── Voxel-downsample source scan before MAD-tree construction ───────────
    // One representative point per source_voxel_size-cell eliminates redundant
    // points that would land in the same MAD-tree leaf, reducing buildMADTree
    // PCA cost without hurting correspondence quality (source normals are not
    // used in ICP — only target leaf normals matter).
    // voxelDownsample() was compiled in voxel_grid.hpp but previously unused.
    const PointCloud source = (cfg_.source_voxel_size > 0.0)
                              ? voxelDownsample(filtered, cfg_.source_voxel_size)
                              : filtered;

    // ── Build source MAD-tree from current scan (sensor frame) ──────────────
    std::vector<Vec3> src_pts(source.begin(), source.end());
    // unique_ptr owns the tree: both early-return and normal paths are leak-safe.
    std::unique_ptr<MADTree> source_root(buildMADTree(src_pts, cfg_.b_max, cfg_.b_min));
    if (!source_root || map_.empty()) {
        ++n_scans_;
        result.pose = T_curr_;
        return result;
    }

    std::vector<MADTree*> src_leaves;
    source_root->getLeavesMut(src_leaves);
    const std::size_t total_src_leaves = src_leaves.size();
    result.num_source_leaves = total_src_leaves;

    // ── ICP ─────────────────────────────────────────────────────────────────
    IcpConfig icp_cfg;
    icp_cfg.max_iterations      = cfg_.max_iterations;
    icp_cfg.delta_chi_eps       = cfg_.delta_chi_eps;
    icp_cfg.b_max               = cfg_.b_max;
    icp_cfg.b_min               = cfg_.b_min;
    icp_cfg.b_ratio             = cfg_.b_ratio;
    icp_cfg.rho_ker             = cfg_.rho_ker;
    icp_cfg.min_correspondences = cfg_.min_correspondences;

    const Pose             T_pred      = predictPose();
    std::vector<MADTree*>  target_trees = map_.trees();
    const IcpResult        icp_res     = icpAlign(target_trees, src_leaves,
                                                   total_src_leaves, T_pred, icp_cfg);

    // ── Health gate ─────────────────────────────────────────────────────────
    // Accept when ICP found enough correspondences, the pose is finite, and chi
    // is below a loose sanity limit.  Inlier ratio is intentionally NOT gated
    // here: after a turn, the robot sees mostly new environment so inlier can
    // legitimately drop to 20-30% while chi stays low (< 0.1 m) — that is a
    // valid alignment.  Gating on inlier causes cascade failures (pose freezes
    // → worse prediction → worse ICP → still frozen).  Chi is the right
    // discriminant: a truly bad alignment has BOTH low inlier AND high chi.
    const bool healthy =
        icp_res.num_correspondences >= cfg_.min_correspondences &&
        icp_res.pose.matrix().allFinite()                       &&
        icp_res.mean_chi            <  cfg_.max_chi_health;

    if (healthy) {
        T_curr_ = icp_res.pose;
    }
    // When not healthy: keep T_curr_ unchanged.
    // Either way, push T_curr_ to the velocity window so the estimator
    // continues to track motion (or correctly learns zero velocity when frozen).
    vel_.push(T_curr_);
    vel_.oneRound();
    ++n_scans_;

    // ── Frame buffer update ──────────────────────────────────────────────────
    if (healthy) {
        // Pre-transform filtered scan to world frame.
        const Mat3 R = T_curr_.linear();
        const Vec3 t = T_curr_.translation();
        PointCloud world_pts;
        world_pts.reserve(filtered.size());
        for (const Vec3& p : filtered) world_pts.push_back(R * p + t);

        frame_buffer_.push_back({T_curr_, icp_res.H_final, std::move(world_pts)});
        if ((int)frame_buffer_.size() > cfg_.frame_window)
            frame_buffer_.pop_front();
    }

    // ── Map update (best-H keyframe selection) ───────────────────────────────
    // Trigger: inlier_ratio dropped below p_th (robot moved to new territory)
    // or the map is empty. Instead of always adding the current frame, pick
    // the frame from the buffer with the highest det(H) — the most
    // geometrically constrained / observable pose.
    const bool want_keyframe =
        healthy &&
        (icp_res.inlier_ratio < cfg_.p_th || map_.empty()) &&
        icp_res.mean_chi < cfg_.max_chi_keyframe &&
        !frame_buffer_.empty();

    if (want_keyframe) {
        // Find best frame by det(H).
        std::size_t best_idx = 0;
        double best_det = -1.0;
        for (std::size_t i = 0; i < frame_buffer_.size(); ++i) {
            const double d = frame_buffer_[i].H.determinant();
            if (d > best_det) { best_det = d; best_idx = i; }
        }
        auto best_world_pts = frame_buffer_[best_idx].world_pts;
        map_.addKeyframe(std::move(best_world_pts));
        // Remove used frame from buffer to avoid re-adding it.
        frame_buffer_.erase(frame_buffer_.begin() + static_cast<int>(best_idx));
        result.new_keyframe = true;
    }

    result.pose                = T_curr_;
    result.icp_healthy         = healthy;
    result.icp_converged       = icp_res.converged;
    result.icp_iterations      = icp_res.iterations;
    result.num_correspondences = icp_res.num_correspondences;
    result.inlier_ratio        = icp_res.inlier_ratio;
    result.mean_chi            = icp_res.mean_chi;

    const Vec3& t3 = T_curr_.translation();
    std::printf("[scan %4d] in=%zu flt=%zu ds=%zu src=%zu corr=%zu "
                "inlier=%.2f chi=%.4f conv=%c hlth=%c kf=%c "
                "pos=%.3f,%.3f,%.3f\n",
                n_scans_,
                result.num_input_points,
                filtered.size(),
                source.size(),
                result.num_source_leaves,
                result.num_correspondences,
                result.inlier_ratio,
                result.mean_chi,
                result.icp_converged ? 'Y' : 'N',
                result.icp_healthy   ? 'Y' : 'N',
                result.new_keyframe  ? 'Y' : 'N',
                t3.x(), t3.y(), t3.z());

    return result;
}

}  // namespace madodom
