#include "madodom_core/mad_tree.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cassert>
#include <cstring>

namespace madodom {

MADTree::~MADTree() {
    delete left_;
    delete right_;
}

// ── internal helpers ────────────────────────────────────────────────────────

static void computeMeanAndCovariance(Vec3& mean, Mat3& cov,
                                     const std::vector<Vec3>& pts,
                                     int begin, int end) {
    mean = Vec3::Zero();
    const int n = end - begin;
    for (int i = begin; i < end; ++i) mean += pts[i];
    mean /= static_cast<double>(n);

    cov = Mat3::Zero();
    for (int i = begin; i < end; ++i) {
        const Vec3 d = pts[i] - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<double>(n);
}

// Extent of pts[begin..end) projected onto each eigenvector column.
// bbox_(i) = range along eigenvectors_.col(i), ordered ascending eigenvalue.
static Vec3 computeBBox(const Vec3& mean, const Mat3& evecs,
                        const std::vector<Vec3>& pts, int begin, int end) {
    Vec3 lo = Vec3::Constant( std::numeric_limits<double>::max());
    Vec3 hi = Vec3::Constant(-std::numeric_limits<double>::max());
    for (int i = begin; i < end; ++i) {
        const Vec3 proj = evecs.transpose() * (pts[i] - mean);
        lo = lo.cwiseMin(proj);
        hi = hi.cwiseMax(proj);
    }
    return (hi - lo).cwiseMax(0.0);
}

// ── build ───────────────────────────────────────────────────────────────────

void MADTree::build(std::vector<Vec3>& pts, int begin, int end,
                    double b_max, double b_min,
                    MADTree* parent, const Vec3* inherited_normal) {
    parent_     = parent;
    num_points_ = end - begin;

    Vec3 centroid;
    Mat3 cov;
    computeMeanAndCovariance(centroid, cov, pts, begin, end);

    Eigen::SelfAdjointEigenSolver<Mat3> es;
    es.computeDirect(cov);
    eigenvectors_ = es.eigenvectors();           // cols sorted ascending eigenvalue
    bbox_         = computeBBox(centroid, eigenvectors_, pts, begin, end);

    // ── LEAF condition: principal extent is smaller than b_max ──────────────
    if (bbox_(2) < b_max || num_points_ < 4) {
        // Normal inheritance: if ancestor was a thin plane (bbox(0) < b_min),
        // inherit its normal to avoid unreliable PCA from few/flat points.
        if (inherited_normal) {
            eigenvectors_.col(0) = *inherited_normal;
        } else if (num_points_ < 3) {
            // Walk up to find a reliable ancestor normal.
            MADTree* node = parent_;
            while (node && node->num_points_ < 3) node = node->parent_;
            if (node) eigenvectors_.col(0) = node->eigenvectors_.col(0);
        }

        // Leaf mean = the point in the cluster nearest the centroid.
        // Avoids centroid falling off the surface.
        double best_d2 = std::numeric_limits<double>::max();
        Vec3   best_p  = centroid;
        for (int i = begin; i < end; ++i) {
            const double d2 = (pts[i] - centroid).squaredNorm();
            if (d2 < best_d2) { best_d2 = d2; best_p = pts[i]; }
        }
        mean_ = best_p;
        return;
    }

    // Internal node: store centroid as mean (used for split queries).
    mean_ = centroid;

    // Detect thin planes: if the smallest bbox dimension < b_min, propagate
    // this node's normal downward so thin leaves inherit it.
    const Vec3* child_normal = inherited_normal;
    Vec3 this_normal;
    if (!child_normal && bbox_(0) < b_min) {
        this_normal  = eigenvectors_.col(0);
        child_normal = &this_normal;
    }

    // Split along principal axis (largest eigenvector = col(2)).
    const Vec3& split_dir = eigenvectors_.col(2);
    const int mid_raw = std::partition(pts.begin() + begin, pts.begin() + end,
                            [&](const Vec3& p) {
                                return (p - mean_).dot(split_dir) < 0.0;
                            }) - pts.begin();

    // Degenerate split (all points on one side): force ~equal halves.
    int mid = mid_raw;
    if (mid == begin || mid == end) mid = (begin + end) / 2;

    left_  = new MADTree();
    right_ = new MADTree();
    left_ ->build(pts, begin, mid, b_max, b_min, this, child_normal);
    right_->build(pts, mid,   end, b_max, b_min, this, child_normal);
}

// ── query ────────────────────────────────────────────────────────────────────

const MADTree* MADTree::bestMatchingLeafFast(const Vec3& query) const {
    const MADTree* node = this;
    while (node->left_ || node->right_) {
        const Vec3& split_dir = node->eigenvectors_.col(2);
        const bool  go_left   = (query - node->mean_).dot(split_dir) < 0.0;
        const MADTree* next   = go_left ? node->left_ : node->right_;
        if (!next) next = go_left ? node->right_ : node->left_;  // fallback
        node = next;
    }
    return node;
}

// ── traversal ────────────────────────────────────────────────────────────────

void MADTree::getLeaves(std::vector<const MADTree*>& out) const {
    if (isLeaf()) { out.push_back(this); return; }
    if (left_)  left_ ->getLeaves(out);
    if (right_) right_->getLeaves(out);
}

void MADTree::getLeavesMut(std::vector<MADTree*>& out) {
    if (isLeaf()) { out.push_back(this); return; }
    if (left_)  left_ ->getLeavesMut(out);
    if (right_) right_->getLeavesMut(out);
}

void MADTree::resetMatched() {
    matched_ = false;
    if (left_)  left_ ->resetMatched();
    if (right_) right_->resetMatched();
}

// ── transform ────────────────────────────────────────────────────────────────

void MADTree::applyTransform(const Mat3& R, const Vec3& t) {
    mean_        = R * mean_ + t;
    eigenvectors_ = R * eigenvectors_;   // normals and split dirs rotate
    if (left_)  left_ ->applyTransform(R, t);
    if (right_) right_->applyTransform(R, t);
}

// ── factory ──────────────────────────────────────────────────────────────────

MADTree* buildMADTree(std::vector<Vec3>& pts, double b_max, double b_min) {
    if (pts.empty()) return nullptr;
    auto* root = new MADTree();
    root->build(pts, 0, static_cast<int>(pts.size()), b_max, b_min);
    return root;
}

}  // namespace madodom
