#pragma once

// MAD-tree: Binary Space Partition tree where each node splits along the
// principal axis (largest eigenvector of the local covariance).  Each leaf
// stores:
//   mean_        – the point in the cluster closest to the centroid
//   eigenvectors_– columns sorted ascending by eigenvalue;
//                  col(0) = surface normal (least-variance direction)
//   bbox_        – extents along each eigenvector [m]; bbox_(2) is the largest
//
// Correspondence search is a greedy descent: always follow the child on the
// same side as the query.  O(log N), no backtracking.
//
// Memory: raw child pointers owned by parent.  Delete the root; children are
// freed recursively by the destructor.

#include <limits>
#include <vector>

#include "madodom_core/types.hpp"

namespace madodom {

struct MADTree {
    Vec3 mean_          = Vec3::Zero();
    Mat3 eigenvectors_  = Mat3::Identity();
    Vec3 bbox_          = Vec3::Zero();
    bool matched_       = false;
    int  num_points_    = 0;

    MADTree* left_   = nullptr;
    MADTree* right_  = nullptr;
    MADTree* parent_ = nullptr;

    MADTree() = default;
    ~MADTree();

    MADTree(const MADTree&)            = delete;
    MADTree& operator=(const MADTree&) = delete;

    // Recursively build from pts[begin..end).
    //   b_max            – max leaf extent in the principal direction [m]
    //   b_min            – thickness below which normal is inherited from ancestor
    //   inherited_normal – non-null when a thin ancestor's normal should propagate
    void build(std::vector<Vec3>& pts, int begin, int end,
               double b_max, double b_min,
               MADTree* parent          = nullptr,
               const Vec3* inherited_normal = nullptr);

    // Greedy-descent nearest-leaf query.
    const MADTree* bestMatchingLeafFast(const Vec3& query,
                                        double search_radius2 = 1e18) const;

    // Append all leaf pointers (no children) to `out`.
    void getLeaves(std::vector<const MADTree*>& out) const;
    void getLeavesMut(std::vector<MADTree*>& out);

    // Clear matched_ on every leaf.
    void resetMatched();

    // Apply rigid transform in-place (world-frame the whole tree in O(N)).
    void applyTransform(const Mat3& R, const Vec3& t);

    bool isLeaf() const { return !left_ && !right_; }

    // Surface normal = smallest eigenvector.
    Vec3 normal() const { return eigenvectors_.col(0); }
};

// Build a new MADTree from pts (modifies ordering inside pts for in-place
// partitioning).  Caller owns the returned pointer.
MADTree* buildMADTree(std::vector<Vec3>& pts, double b_max, double b_min);

}  // namespace madodom
