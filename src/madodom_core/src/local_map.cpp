#include "madodom_core/local_map.hpp"

namespace madodom {

LocalMap::LocalMap(const LocalMapConfig& cfg) : cfg_(cfg) {}

LocalMap::~LocalMap() { clear(); }

void LocalMap::addKeyframe(std::vector<Vec3> pts) {
    if (pts.empty()) return;

    MADTree* root = buildMADTree(pts, cfg_.b_max, cfg_.b_min);
    if (!root) return;

    trees_.push_back(root);

    // Prune oldest when over capacity.
    while (static_cast<int>(trees_.size()) > cfg_.num_keyframes) {
        delete trees_.front();
        trees_.pop_front();
    }
}

std::vector<MADTree*> LocalMap::trees() {
    return std::vector<MADTree*>(trees_.begin(), trees_.end());
}

void LocalMap::clear() {
    for (MADTree* t : trees_) delete t;
    trees_.clear();
}

}  // namespace madodom
