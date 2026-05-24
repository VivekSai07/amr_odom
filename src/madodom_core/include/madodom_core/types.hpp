#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

namespace madodom {

using Scalar     = double;
using Vec3       = Eigen::Vector3d;
using Vec6       = Eigen::Matrix<double, 6, 1>;
using Mat3       = Eigen::Matrix3d;
using Mat6       = Eigen::Matrix<double, 6, 6>;
using Pose       = Eigen::Isometry3d;
using PointCloud = std::vector<Vec3>;

// 3×3 skew-symmetric matrix for v.
inline Mat3 skew(const Vec3& v) {
    Mat3 S;
    S <<  0.0,  -v(2),  v(1),
          v(2),  0.0,  -v(0),
         -v(1),  v(0),  0.0;
    return S;
}

// Rodrigues: se(3) rotation vector → SO(3).
inline Mat3 expMapSO3(const Vec3& omega) {
    const double theta = omega.norm();
    if (theta < 1e-10) return Mat3::Identity();
    const Vec3   axis  = omega / theta;
    const double c     = std::cos(theta);
    const double s     = std::sin(theta);
    return c * Mat3::Identity() + (1.0 - c) * axis * axis.transpose() + s * skew(axis);
}

// SO(3) → rotation vector (angle-axis).
inline Vec3 logMapSO3(const Mat3& R) {
    const double cos_theta = std::clamp(0.5 * (R.trace() - 1.0), -1.0, 1.0);
    const double theta     = std::acos(cos_theta);
    if (std::abs(theta) < 1e-10) return Vec3::Zero();
    const double s = std::sin(theta);
    // Near π: use symmetric formula to avoid division by ~0 sin.
    if (std::abs(s) < 1e-6) {
        const Vec3 ax = Vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        return theta * ax.normalized();
    }
    return (theta / (2.0 * s)) * Vec3(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
}

}  // namespace madodom
