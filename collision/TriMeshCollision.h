//
// Created by tianyan on 1/20/26.
//

#ifndef TAIYI_TRIMESHCOLLISION_H
#define TAIYI_TRIMESHCOLLISION_H

#include "Types.h"

inline Vec3 minv(const Vec3& a,const Vec3& b){ return {std::min(a.x(),b.x()),std::min(a.y(),b.y()),std::min(a.z(),b.z())}; }
inline Vec3 maxv(const Vec3& a,const Vec3& b){ return {std::max(a.x(),b.x()),std::max(a.y(),b.y()),std::max(a.z(),b.z())}; }

struct AABB {
    Vec3 lo{ +std::numeric_limits<float>::infinity(),
             +std::numeric_limits<float>::infinity(),
             +std::numeric_limits<float>::infinity() };
    Vec3 hi{ -std::numeric_limits<float>::infinity(),
             -std::numeric_limits<float>::infinity(),
             -std::numeric_limits<float>::infinity() };

    void expand(const Vec3& p){ lo = minv(lo,p); hi = maxv(hi,p); }
    void expand(const AABB& b){ lo = minv(lo,b.lo); hi = maxv(hi,b.hi); }

    [[nodiscard]] Vec3 centroid() const { return (lo + hi) * 0.5f; }

    [[nodiscard]] bool overlaps(const AABB& b) const {
        return (lo.x() <= b.hi.x() && hi.x() >= b.lo.x()) &&
               (lo.y() <= b.hi.y() && hi.y() >= b.lo.y()) &&
               (lo.z() <= b.hi.z() && hi.z() >= b.lo.z());
    }
};

class AABBTree {

public:

    struct Node {
        AABB box;
        int left = -1;
        int right = -1;
        int first = -1;     // index into prim_indices_
        int count = -1;     // number of primitives in leaf
        bool is_leaf = false;
    };

    void build(const std::vector<AABB>& prim_boxes, int leaf_size = 1);

    // After user updates prim boxes array in-place, call refit to refresh internal node boxes
    void refit(const std::vector<AABB>& prim_boxes);


    template<class F>
    void query_aabb(const AABB& q, F&& on_hit_prim) const {
        if (root_ < 0) return;
        int stack[64];
        int sp = 0;
        stack[sp++] = root_;
        while (sp) {
            const int n = stack[--sp];
            const auto&[box, left, right, first, count, is_leaf] = nodes_[n];
            if (!box.overlaps(q)) continue;
            if (is_leaf) {
                for (int i = 0; i < count; ++i) {
                    int prim_id = prim_indices_[first + i];
                    on_hit_prim(prim_id);
                }
            } else {
                stack[sp++] = left;
                stack[sp++] = right;
            }
        }
    }

    template <class F>
    void traverse(F&& fn) const {
        if (root_ < 0) return;
        int stack_node[128];
        int stack_depth[128];
        int sp = 0;
        stack_node[sp] = root_;
        stack_depth[sp] = 0;
        ++sp;

        while (sp) {
            --sp;
            int n = stack_node[sp];
            int depth = stack_depth[sp];
            const Node& nd = nodes_[n];

            // fn(node_index, node, depth)
            fn(n, nd, depth);

            if (!nd.is_leaf) {
                // push children
                stack_node[sp] = nd.left;  stack_depth[sp] = depth + 1; ++sp;
                stack_node[sp] = nd.right; stack_depth[sp] = depth + 1; ++sp;
            }
        }
    }

private:

    int build_recursive(int first, int count);

    AABB refit_recursive(int node_id);

    const std::vector<AABB>* prim_boxes_ = nullptr;
    int leaf_size_ = 1;
    int root_ = -1;
    std::vector<int> prim_indices_;
    std::vector<Node> nodes_;
};


class TriMeshCollisionDetector {
public:
    struct CollisionInfo {
        // flattened pairs (v, tri) stored as 2-int per entry, CSR by vertex
        std::vector<int> vertex_colliding_triangles;
        std::vector<int> vertex_colliding_triangles_offsets; // size = particle_count+1
        std::vector<int> vertex_colliding_triangles_count;   // size = particle_count
        std::vector<float> vertex_colliding_triangles_min_dist;

        // flattened pairs (e, e2) stored as 2-int per entry, CSR by edge
        std::vector<int> edge_colliding_edges;
        std::vector<int> edge_colliding_edges_offsets; // size = edge_count+1
        std::vector<int> edge_colliding_edges_count;   // size = edge_count
        std::vector<float> edge_colliding_edges_min_dist;

        // overflow flags like Newton (0/1)
        int vertex_overflow = 0;
        int tri_overflow = 0;   // not used in this minimal CPU version
        int edge_overflow = 0;
    };


};





#endif //TAIYI_TRIMESHCOLLISION_H