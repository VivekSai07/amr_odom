#pragma once

// Range filter + first-point-per-voxel downsampling (double precision).
// std::floor handles negative coordinates correctly (unlike integer truncation).

#include <cmath>
#include <cstdint>
#include <unordered_set>

#include "madodom_core/types.hpp"

namespace madodom {

struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const noexcept {
        return static_cast<std::size_t>(k.x) * 73856093u ^
               static_cast<std::size_t>(k.y) * 19349663u ^
               static_cast<std::size_t>(k.z) * 83492791u;
    }
};

inline VoxelKey voxelIndex(const Vec3& p, double voxel_size) {
    return VoxelKey{
        static_cast<int32_t>(std::floor(p.x() / voxel_size)),
        static_cast<int32_t>(std::floor(p.y() / voxel_size)),
        static_cast<int32_t>(std::floor(p.z() / voxel_size)),
    };
}

// Keep first point per voxel. Empty or non-positive voxel_size → empty output.
inline PointCloud voxelDownsample(const PointCloud& in, double voxel_size) {
    PointCloud out;
    if (in.empty() || voxel_size <= 0.0) return out;
    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    const std::size_t guess = std::max<std::size_t>(in.size() / 4, 64);
    seen.reserve(guess);
    out.reserve(guess);
    for (const auto& p : in) {
        if (seen.insert(voxelIndex(p, voxel_size)).second) out.push_back(p);
    }
    return out;
}

// Range filter: keep points with squared range in [r_min², r_max²].
inline PointCloud filterByRange(const PointCloud& in, double r_min, double r_max) {
    PointCloud out;
    out.reserve(in.size());
    const double r2_min = r_min * r_min;
    const double r2_max = r_max * r_max;
    for (const auto& p : in) {
        const double d2 = p.squaredNorm();
        if (d2 >= r2_min && d2 <= r2_max) out.push_back(p);
    }
    return out;
}

}  // namespace madodom
