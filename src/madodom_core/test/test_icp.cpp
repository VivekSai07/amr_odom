#include <gtest/gtest.h>
#include "madodom_core/icp.hpp"
#include "madodom_core/mad_tree.hpp"
#include <random>
#include <cmath>

using namespace madodom;

// ── Helpers ───────────────────────────────────────────────────────────────────

static std::vector<Vec3> makeRoomCloud(int n = 600) {
    // Simulate a simple "room": floor + 4 walls as flat patches.
    std::mt19937 rng(13);
    std::uniform_real_distribution<double> u(-3.0, 3.0);
    std::normal_distribution<double> noise(0.0, 0.005);
    std::vector<Vec3> pts;
    pts.reserve(n);
    // Floor (z=0)
    for (int i = 0; i < n / 5; ++i)
        pts.emplace_back(u(rng), u(rng), noise(rng));
    // Ceiling (z=2.5)
    for (int i = 0; i < n / 5; ++i)
        pts.emplace_back(u(rng), u(rng), 2.5 + noise(rng));
    // Left/right walls (y=±3)
    for (int i = 0; i < n / 5; ++i) {
        pts.emplace_back(u(rng), 3.0 + noise(rng),  u(rng));
        pts.emplace_back(u(rng), -3.0 + noise(rng), u(rng));
    }
    // Front/back walls (x=±3)
    for (int i = 0; i < n / 5; ++i) {
        pts.emplace_back(3.0 + noise(rng),  u(rng), u(rng));
        pts.emplace_back(-3.0 + noise(rng), u(rng), u(rng));
    }
    return pts;
}

static IcpConfig defaultCfg() {
    IcpConfig c;
    c.max_iterations      = 50;
    c.delta_chi_eps       = 1e-8;
    c.b_max               = 0.4;
    c.b_min               = 0.05;
    c.b_ratio             = 0.02;
    c.rho_ker             = 0.1;
    c.min_correspondences = 5;
    return c;
}

// ── Test 1: align cloud with itself → identity ─────────────────────────────────
TEST(ICP, SelfAlignIsIdentity) {
    auto room = makeRoomCloud(800);

    // Build target keyframe.
    std::vector<Vec3> tgt_pts = room;
    MADTree* tgt_root = buildMADTree(tgt_pts, 0.4, 0.05);
    ASSERT_NE(tgt_root, nullptr);
    std::vector<MADTree*> target_trees = {tgt_root};

    // Build source from same cloud (different copy → different in-place order).
    std::vector<Vec3> src_pts = room;
    MADTree* src_root = buildMADTree(src_pts, 0.4, 0.05);
    ASSERT_NE(src_root, nullptr);
    std::vector<MADTree*> src_leaves;
    src_root->getLeavesMut(src_leaves);

    const Pose init = Pose::Identity();
    const IcpResult res = icpAlign(target_trees, src_leaves,
                                   src_leaves.size(), init, defaultCfg());

    // Translation error < 5 mm, rotation error < 0.5°.
    EXPECT_LT(res.pose.translation().norm(), 0.005)
        << "Self-align translation error too large";
    const double angle_err =
        Eigen::AngleAxisd(res.pose.linear()).angle() * 180.0 / M_PI;
    EXPECT_LT(angle_err, 0.5)
        << "Self-align rotation error too large: " << angle_err << "°";
    EXPECT_GE(res.num_correspondences, 5u);

    delete src_root;
    delete tgt_root;
}

// ── Test 2: align shifted cloud → recovers the shift ──────────────────────────
TEST(ICP, RecoverKnownTranslation) {
    auto room = makeRoomCloud(800);

    // Target = original room.
    std::vector<Vec3> tgt_pts = room;
    MADTree* tgt_root = buildMADTree(tgt_pts, 0.4, 0.05);
    ASSERT_NE(tgt_root, nullptr);
    std::vector<MADTree*> target_trees = {tgt_root};

    // Source = room shifted by (0.05, 0.0, 0.0) — 5 cm.
    const Vec3 true_t(0.05, 0.0, 0.0);
    std::vector<Vec3> src_pts_shifted;
    src_pts_shifted.reserve(room.size());
    for (const Vec3& p : room) src_pts_shifted.push_back(p + true_t);

    std::vector<Vec3> src_build = src_pts_shifted;
    MADTree* src_root = buildMADTree(src_build, 0.4, 0.05);
    ASSERT_NE(src_root, nullptr);

    // Source tree is in world frame (pre-shifted). We want ICP starting from
    // identity to recover the pose that maps source → target, which should
    // be T = -true_t translation (or equivalently the pose T such that
    // T * source ≈ target).
    // We start warm from identity pose. Source leaves are already in "world"
    // but offset — ICP should find T ≈ -true_t.
    std::vector<MADTree*> src_leaves;
    src_root->getLeavesMut(src_leaves);

    const IcpResult res = icpAlign(target_trees, src_leaves,
                                   src_leaves.size(), Pose::Identity(), defaultCfg());

    // The converged translation should ≈ -true_t.
    const Vec3 t_err = res.pose.translation() - (-true_t);
    EXPECT_LT(t_err.norm(), 0.01)
        << "Known-translation recovery error: " << res.pose.translation().transpose();

    delete src_root;
    delete tgt_root;
}

// ── Test 3: align rotated cloud → recovers rotation ──────────────────────────
TEST(ICP, RecoverKnownRotation) {
    auto room = makeRoomCloud(800);

    std::vector<Vec3> tgt_pts = room;
    MADTree* tgt_root = buildMADTree(tgt_pts, 0.4, 0.05);
    ASSERT_NE(tgt_root, nullptr);
    std::vector<MADTree*> target_trees = {tgt_root};

    // Source = room rotated by 3° around Z.
    const double theta = 3.0 * M_PI / 180.0;
    const Mat3 R_true =
        Eigen::AngleAxisd(theta, Vec3::UnitZ()).toRotationMatrix();
    std::vector<Vec3> src_pts_rot;
    src_pts_rot.reserve(room.size());
    for (const Vec3& p : room) src_pts_rot.push_back(R_true * p);

    std::vector<Vec3> src_build = src_pts_rot;
    MADTree* src_root = buildMADTree(src_build, 0.4, 0.05);
    ASSERT_NE(src_root, nullptr);
    std::vector<MADTree*> src_leaves;
    src_root->getLeavesMut(src_leaves);

    const IcpResult res = icpAlign(target_trees, src_leaves,
                                   src_leaves.size(), Pose::Identity(), defaultCfg());

    // Recovered rotation should be ≈ R_true^{-1}.
    const Mat3 R_err = res.pose.linear() * R_true;  // should ≈ I
    const double angle_err =
        Eigen::AngleAxisd(R_err).angle() * 180.0 / M_PI;
    EXPECT_LT(angle_err, 1.0)
        << "Known-rotation recovery error: " << angle_err << "°";

    delete src_root;
    delete tgt_root;
}
