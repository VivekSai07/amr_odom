#include "madodom_core/icp.hpp"

#include <cmath>
#include <limits>

namespace madodom {

IcpResult icpAlign(const std::vector<MADTree*>& target_trees,
                   std::vector<MADTree*>&        source_leaves,
                   std::size_t                   total_source_leaves,
                   const Pose&                   initial_pose,
                   const IcpConfig&              cfg) {
    IcpResult result;
    result.pose = initial_pose;

    if (source_leaves.empty() || target_trees.empty()) return result;

    const double rho_ker = std::sqrt(cfg.rho_ker);  // Huber threshold in metres

    Pose T = initial_pose;
    double last_chi = std::numeric_limits<double>::max();

    std::size_t last_n_corr = 0;
    Mat6 H_final = Mat6::Zero();

    for (int iter = 0; iter < cfg.max_iterations; ++iter) {
        // Reset matched flags every iteration so the final count reflects the
        // last completed pass (not a stale flag from an earlier one).
        for (MADTree* l : source_leaves) l->matched_ = false;

        Mat6 H = Mat6::Zero();
        Vec6 b = Vec6::Zero();
        double chi_sum  = 0.0;
        std::size_t n_corr = 0;
        const Mat3 R = T.linear();

        for (MADTree* src : source_leaves) {
            const Vec3 s = src->mean_;              // source leaf mean, sensor frame
            const Vec3 p = T * s;                   // transformed to world frame

            // ── find best matching leaf across ALL keyframe trees ────────────
            const MADTree* best_leaf = nullptr;
            double best_dist2        = std::numeric_limits<double>::max();
            for (MADTree* kf : target_trees) {
                const MADTree* candidate = kf->bestMatchingLeafFast(p);
                const double   d2        = (p - candidate->mean_).squaredNorm();
                if (d2 < best_dist2) { best_dist2 = d2; best_leaf = candidate; }
            }
            if (!best_leaf) continue;

            // ── 1. Distance gate (adaptive radius) ───────────────────────────
            const double r_max = cfg.b_max + cfg.b_ratio * s.norm();
            if (std::sqrt(best_dist2) > r_max) continue;

            // ── Point-to-plane residual and Jacobian ─────────────────────────
            const Vec3 n = best_leaf->normal();   // world-frame normal
            const double e = (p - best_leaf->mean_).dot(n);

            // J = [ n^T R | -n^T R [s]× ]  (1×6)
            const Vec3 nR = R.transpose() * n;    // R^T n (reuse below)
            Eigen::Matrix<double, 1, 6> J;
            J.block<1, 3>(0, 0) = nR.transpose();
            J.block<1, 3>(0, 3) = -(R.transpose() * n).transpose() * skew(s);
            // Equivalently: J(0,3..5) = -nR^T [s]× but we write it directly:
            // J.block<1,3>(0,3) = (s.cross(nR)).transpose()  — same result.

            // ── 2. Huber weight ──────────────────────────────────────────────
            const double abs_e   = std::abs(e);
            const double w_huber = (abs_e <= rho_ker) ? 1.0 : rho_ker / abs_e;

            // ── 3. Flatness weight (prefer thin planar leaves) ───────────────
            const double wf    = std::max(0.0, 1.0 - best_leaf->bbox_(0) / cfg.b_max);
            const double scale = w_huber * wf * wf;

            H   += scale * J.transpose() * J;
            b   += scale * J.transpose() * e;
            chi_sum += abs_e;
            ++n_corr;
            src->matched_ = true;  // flag is reset at start of each iter
        }

        last_n_corr = n_corr;
        H_final = H;  // capture last H before GN step
        if (n_corr < cfg.min_correspondences) break;

        // ── Gauss-Newton step ─────────────────────────────────────────────────
        const Vec6 dx  = H.ldlt().solve(-b);
        Pose       dX  = Pose::Identity();
        dX.linear()      = expMapSO3(dx.tail<3>());
        dX.translation() = dx.head<3>();
        T = T * dX;                         // right perturbation

        result.iterations = iter + 1;

        // ── Convergence check ─────────────────────────────────────────────────
        const double chi_avg = chi_sum / static_cast<double>(n_corr);
        if (std::abs(last_chi - chi_avg) < cfg.delta_chi_eps) {
            result.converged = true;
            break;
        }
        last_chi = chi_avg;
    }

    // matched_ flags reflect the last completed ICP iteration.
    std::size_t matched = last_n_corr;

    result.pose              = T;
    result.H_final           = H_final;
    result.num_correspondences = matched;
    result.inlier_ratio      = (total_source_leaves > 0)
                               ? static_cast<double>(matched) / total_source_leaves
                               : 0.0;
    result.mean_chi          = (result.iterations > 0) ? last_chi : 0.0;
    return result;
}

}  // namespace madodom
