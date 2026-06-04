#pragma once

// Sliding-window keyframe map.
//
// Each keyframe is a MADTree built from that scan's filtered points
// transformed into the world frame.  The map is a bounded deque of trees;
// the oldest keyframe is pruned when the limit is exceeded.
//
// Keyframe insertion is triggered by the odometry layer when the ICP inlier
// ratio drops below p_th (the scan has moved into terrain not well-covered
// by the existing map).

#include <deque>
#include <memory>
#include <vector>

#include "madodom_core/mad_tree.hpp"
#include "madodom_core/types.hpp"

namespace madodom {

struct LocalMapConfig {
    int    num_keyframes = 8;     // maximum keyframes kept
    double p_th          = 0.8;   // inlier-ratio threshold for new keyframe
    double b_max         = 0.6;   // tree leaf-size parameter [m]
    double b_min         = 0.1;   // normal-inheritance threshold [m]
};

class LocalMap {
public:
    explicit LocalMap(const LocalMapConfig& cfg);
    ~LocalMap();

    LocalMap(const LocalMap&)            = delete;
    LocalMap& operator=(const LocalMap&) = delete;

    // Add a new keyframe built from world-frame points.
    // pts is consumed (reordered internally for in-place partitioning).
    void addKeyframe(std::vector<Vec3> pts);

    // True when the map has no keyframes yet.
    bool empty() const { return trees_.empty(); }

    // Pointers to all keyframe root trees (world frame).
    std::vector<MADTree*> trees();

    // Number of keyframes currently stored.
    std::size_t size() const { return trees_.size(); }

    void clear();

private:
    LocalMapConfig cfg_;
    std::deque<std::unique_ptr<MADTree>> trees_;
};

}  // namespace madodom
