#include "madodom_core/local_map.hpp"

namespace madodom {

LocalMap::LocalMap(const LocalMapConfig& cfg) : cfg_(cfg) {}

// unique_ptr destructor recursively frees the entire tree; no manual delete.
LocalMap::~LocalMap() = default;

void LocalMap::addKeyframe(std::vector<Vec3> pts) {
    if (pts.empty()) return;

    MADTree* root = buildMADTree(pts, cfg_.b_max, cfg_.b_min);
    if (!root) return;

    trees_.emplace_back(root);  // unique_ptr takes ownership of raw pointer

    // Prune oldest when over capacity — unique_ptr destructor handles deletion.
    while (static_cast<int>(trees_.size()) > cfg_.num_keyframes)
        trees_.pop_front();
}

std::vector<MADTree*> LocalMap::trees() {
    std::vector<MADTree*> out;
    out.reserve(trees_.size());
    for (const auto& t : trees_) out.push_back(t.get());
    return out;
}

void LocalMap::clear() {
    trees_.clear();  // unique_ptr destructors free all trees automatically
}

}  // namespace madodom
