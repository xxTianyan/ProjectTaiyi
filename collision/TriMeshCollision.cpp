//
// Created by tianyan on 1/20/26.
//

#include "TriMeshCollision.h"

void AABBTree::build(const std::vector<AABB> &prim_boxes, const int leaf_size) {
    prim_boxes_ = &prim_boxes;
    leaf_size_  = std::max(1, leaf_size);

    prim_indices_.resize(static_cast<int>(prim_boxes.size()));
    for (int i = 0; i < static_cast<int>(prim_boxes.size()); ++i) prim_indices_[i] = i;

    nodes_.clear();
    nodes_.reserve(static_cast<int>(prim_boxes.size()) * 2);

    root_ = build_recursive(0, static_cast<int>(prim_boxes.size()));
    // compute root box already done in recursion
}

void AABBTree::refit(const std::vector<AABB> &prim_boxes) {
    prim_boxes_ = &prim_boxes;
    if (root_ < 0) return;
    refit_recursive(root_);
}

int AABBTree::build_recursive(const int first, const int count) {
    Node node;
    node.first = first;
    node.count = count;

    // compute bounds of this range
    AABB aabb;
    for (int i = 0; i < count; ++i) {
        aabb.expand((*prim_boxes_)[prim_indices_[first + i]]);
    }
    node.box = aabb;

    const int this_id = static_cast<int>(nodes_.size());
    nodes_.push_back(node);

    if (count <= leaf_size_) {
        nodes_[this_id].is_leaf = true;
        return this_id;
    }

    // split by centroid along the largest axis
    Vec3 cmin{ +std::numeric_limits<float>::infinity(),
               +std::numeric_limits<float>::infinity(),
               +std::numeric_limits<float>::infinity() };
    Vec3 cmax{ -std::numeric_limits<float>::infinity(),
               -std::numeric_limits<float>::infinity(),
               -std::numeric_limits<float>::infinity() };
    for (int i = 0; i < count; ++i) {
        Vec3 c = (*prim_boxes_)[prim_indices_[first + i]].centroid();
        cmin = minv(cmin, c);
        cmax = maxv(cmax, c);
    }
    Vec3 ext = cmax - cmin;
    int axis = 0;
    if (ext.y() > ext.x()) axis = 1;
    if ((axis==0 ? ext.x() : ext.y()) < ext.z()) axis = 2;

    auto key = [&](const int prim_id) {
        Vec3 c = (*prim_boxes_)[prim_id].centroid();
        return (axis == 0 ? c.x() : (axis == 1 ? c.y() : c.z()));
    };

    const int mid = first + count/2;
    std::nth_element(
        prim_indices_.begin() + first,
        prim_indices_.begin() + mid,
        prim_indices_.begin() + first + count,
        [&](const int a, const int b){ return key(a) < key(b); }
    );

    const int left  = build_recursive(first, mid - first);
    const int right = build_recursive(mid, first + count - mid);

    nodes_[this_id].left = left;
    nodes_[this_id].right = right;
    nodes_[this_id].is_leaf = false;
    return this_id;
}

AABB AABBTree::refit_recursive(const int node_id) {
    if (auto&[box, left, right, first, count, is_leaf] = nodes_[node_id]; is_leaf) {
        AABB b;
        for (int i = 0; i < count; ++i) {
            b.expand((*prim_boxes_)[prim_indices_[first + i]]);
        }
        box = b;
        return b;
    }
    else {
        const AABB bl = refit_recursive(left);
        const AABB br = refit_recursive(right);
        AABB b; b.expand(bl); b.expand(br);
        box = b;
        return b;
    }
}
