#pragma once

// Point-to-plane ICP on SE(3) using MAD-tree correspondences.
//
// Perturbation convention (RIGHT):  T_new = T * exp(δξ)
//   δξ = [δt ; δω]  (translation first, rotation second)
//
// Residual per correspondence:
//   e_i = n_i · (T · s_i − m_i)         n_i = target leaf normal (world frame)
//                                         s_i = source leaf mean  (sensor frame)
//
// Jacobian (1×6):
//   J_i = [ n_i^T R | −n_i^T R [s_i]× ]
//
// Three-tier robust weighting per correspondence (exactly as MAD-ICP):
//   1. Distance gate  : reject if ‖T·s_i − m_i‖ > b_max + b_ratio·‖s_i‖
//   2. Huber          : w_h = min(1, rho_ker / |e_i|)
//   3. Flatness       : w_f = max(0, 1 − bbox0 / b_max)²
//   Combined          : w = w_h · w_f
//
// Solver: Gauss-Newton with LDLT.  Convergence: |Δchi| < delta_chi_eps.

#include <cstddef>
#include <vector>

#include "madodom_core/mad_tree.hpp"
#include "madodom_core/types.hpp"

namespace madodom {

struct IcpConfig {
    int    max_iterations     = 15;
    double delta_chi_eps      = 1e-6;   // stop when |chi_new − chi_old| < this
    double b_max              = 0.6;    // max leaf extent = search radius base [m]
    double b_min              = 0.1;    // flatness weight denominator [m]
    double b_ratio            = 0.02;   // search radius grows by b_ratio·‖s‖
    double rho_ker            = 0.1;    // Huber threshold² (code uses sqrt)
    std::size_t min_correspondences = 10;
};

struct IcpResult {
    Pose        pose             = Pose::Identity();
    Mat6        H_final          = Mat6::Zero();  // information matrix at last ICP iter
    int         iterations       = 0;
    std::size_t num_correspondences = 0;
    double      mean_chi         = 0.0;
    double      inlier_ratio     = 0.0;  // matched source leaves / total
    bool        converged        = false;
};

// Align source leaves (sensor frame) to a set of target keyframe trees (world
// frame).  initial_pose is the warm-start T (sensor→world).
// Source leaves come from a MADTree built from the current scan; target_trees
// is the sliding keyframe window (each already in world frame).
IcpResult icpAlign(const std::vector<MADTree*>& target_trees,
                   std::vector<MADTree*>&        source_leaves,
                   std::size_t                   total_source_leaves,
                   const Pose&                   initial_pose,
                   const IcpConfig&              cfg);

}  // namespace madodom
