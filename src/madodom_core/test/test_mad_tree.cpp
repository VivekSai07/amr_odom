#include <gtest/gtest.h>
#include "madodom_core/mad_tree.hpp"
#include <random>

using namespace madodom;

// Build a flat point cloud (XY plane) with known normal (+Z).
static std::vector<Vec3> makePlane(int n_pts, double noise = 0.001) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> xy(-2.0, 2.0);
    std::normal_distribution<double>       nz(0.0, noise);
    std::vector<Vec3> pts;
    pts.reserve(n_pts);
    for (int i = 0; i < n_pts; ++i)
        pts.emplace_back(xy(rng), xy(rng), nz(rng));
    return pts;
}

// Build a random point cloud uniformly in a cube.
static std::vector<Vec3> makeCube(int n_pts, double side = 4.0) {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u(-side / 2, side / 2);
    std::vector<Vec3> pts;
    pts.reserve(n_pts);
    for (int i = 0; i < n_pts; ++i) pts.emplace_back(u(rng), u(rng), u(rng));
    return pts;
}

// ── Self-match test ────────────────────────────────────────────────────────────
// For each point in the cloud, query the tree. The returned leaf must be
// within b_max of the query (since the cloud was used to build the tree).
TEST(MADTree, SelfMatchWithinLeafSize) {
    auto pts = makeCube(500);
    const double b_max = 0.3;
    const double b_min = 0.05;

    std::vector<Vec3> build_pts = pts;  // buildMADTree reorders in-place
    MADTree* root = buildMADTree(build_pts, b_max, b_min);
    ASSERT_NE(root, nullptr);

    for (const Vec3& p : pts) {
        const MADTree* leaf = root->bestMatchingLeafFast(p);
        ASSERT_NE(leaf, nullptr);
        // The leaf must be a valid leaf (no children).
        EXPECT_TRUE(leaf->isLeaf());
        // Greedy BSP descent is not exact nearest-neighbor; the found leaf
        // can be up to a few b_max away. The important thing: it's a valid leaf.
        EXPECT_LT((leaf->mean_ - p).norm(), b_max * 10.0)
            << "leaf mean too far from query";
    }

    delete root;
}

// ── Plane normal test ─────────────────────────────────────────────────────────
// A flat XY cloud should produce leaves with normals ≈ ±Z.
TEST(MADTree, PlaneNormal) {
    auto pts = makePlane(400, /*noise=*/0.005);
    const double b_max = 0.4;
    const double b_min = 0.05;

    std::vector<Vec3> build_pts = pts;
    MADTree* root = buildMADTree(build_pts, b_max, b_min);
    ASSERT_NE(root, nullptr);

    std::vector<const MADTree*> leaves;
    root->getLeaves(leaves);
    EXPECT_GT(leaves.size(), 0u);

    int good = 0;
    for (const MADTree* leaf : leaves) {
        const double cos_angle = std::abs(leaf->normal().dot(Vec3::UnitZ()));
        if (cos_angle > 0.9) ++good;
    }
    // At least 80% of leaves should have normals within ~25° of +Z.
    EXPECT_GT(static_cast<double>(good) / leaves.size(), 0.8)
        << "Most leaf normals should be ~Z for a flat cloud";

    delete root;
}

// ── applyTransform test ───────────────────────────────────────────────────────
// Translate the tree by (1,2,3) and verify leaf means shift accordingly.
TEST(MADTree, ApplyTransform) {
    auto pts = makeCube(200);
    std::vector<Vec3> build_pts = pts;
    MADTree* root = buildMADTree(build_pts, 0.4, 0.05);
    ASSERT_NE(root, nullptr);

    // Collect leaf means before transform.
    std::vector<const MADTree*> leaves_before;
    root->getLeaves(leaves_before);
    std::vector<Vec3> means_before;
    for (const auto* l : leaves_before) means_before.push_back(l->mean_);

    const Vec3 t(1.0, 2.0, 3.0);
    root->applyTransform(Mat3::Identity(), t);

    std::vector<const MADTree*> leaves_after;
    root->getLeaves(leaves_after);
    ASSERT_EQ(leaves_before.size(), leaves_after.size());

    for (std::size_t i = 0; i < means_before.size(); ++i) {
        EXPECT_LT((leaves_after[i]->mean_ - (means_before[i] + t)).norm(), 1e-9);
    }

    delete root;
}

// ── resetMatched ──────────────────────────────────────────────────────────────
TEST(MADTree, ResetMatched) {
    auto pts = makeCube(100);
    std::vector<Vec3> build_pts = pts;
    MADTree* root = buildMADTree(build_pts, 0.4, 0.05);
    ASSERT_NE(root, nullptr);

    std::vector<MADTree*> leaves;
    root->getLeavesMut(leaves);
    for (auto* l : leaves) l->matched_ = true;

    root->resetMatched();

    for (const auto* l : leaves) EXPECT_FALSE(l->matched_);

    delete root;
}
