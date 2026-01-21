//
// Created by tianyan on 1/20/26.
//

#ifndef TAIYI_TRIMESHCOLLISION_H
#define TAIYI_TRIMESHCOLLISION_H

#include "Types.h"
struct ForceElementAdjacencyInfo;
struct MModel;

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
            }
            else {
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

        // per-triangle: min distance to any colliding vertex , currently no triangle colliding data
        std::vector<float> triangle_colliding_vertices_min_dist; // size = tri_count

        // overflow flags like Newton (0/1)
        int vertex_overflow = 0;
        int tri_overflow = 0;   // not used in this minimal CPU version
        int edge_overflow = 0;
    };

    explicit TriMeshCollisionDetector(const MModel& model, const ForceElementAdjacencyInfo& adj, int vertex_pre_alloc = 8, int vertex_max_alloc = 64, int edge_pre_alloc = 8,
                                      int edge_max_alloc = 64, int leaf_size = 1);

    void collision_detection(const State& state_in);

    Vec3 apply_conservative_bounds(const VertexID v, const Vec3& inertia) const;

    // Optional filtering lists (must be sorted per-vertex/per-edge if you want binary search)
    void set_vertex_triangle_filter_list(const std::vector<int>* list, const std::vector<int>* offsets);

public:
    // max query radius, margin of aabb box of vertex
    float particle_contact_margin = 0.02f;
    // delete primitives that very close to target vertex in rest pose
    float particle_rest_shape_contact_exclusion_radius = 0.2f;
    // parameter to scale conservative bound
    float conservative_bound_relaxation = 0.1f;

private:

    void vertex_triangle_collision_detection(const std::vector<Vec3>& pos, float max_query_radius, float rest_exclusion_radius = 0.0f,
        const std::vector<Vec3>* min_distance_filtering_ref_pos = nullptr);

    /*void edge_edge_collision_detection(const std::vector<Vec3>& pos, float max_query_radius, float min_query_radius = 0.0f,
        const std::vector<Vec3>* min_distance_filtering_ref_pos = nullptr);*/

    void compute_tri_aabbs(const std::vector<Vec3>& pos);

    void compute_edge_aabbs(const std::vector<Vec3>& pos);

    void ensure_capacity_for_vertex_buffers();

    static void compute_offsets(const std::vector<int>& sizes, std::vector<int>& offsets);

    // Build trees from current positions
    void build(const std::vector<Vec3>& pos);

    void rebuild(const std::vector<Vec3>& pos) { build(pos);}

    void refit(const std::vector<Vec3>& pos);

    void compute_particle_conservative_bounds();

private:
    const MModel& model_;
    const ForceElementAdjacencyInfo& adj_;
    int v_pre_, v_max_;
    int e_pre_, e_max_;
    int leaf_size_;

    std::vector<AABB> tri_boxes_;
    std::vector<AABB> edge_boxes_;
    AABBTree bvh_tris_;
    AABBTree bvh_edges_;

    // buffer sizes (Newton-style pre-alloc per primitive)
    std::vector<int> v_buf_sizes_;
    std::vector<int> e_buf_sizes_;

    // Penetration-free state
    std::vector<Vec3>  pos_prev_collision_detection_;
    std::vector<float> particle_conservative_bounds_;

    // filtering list (optional)
    const std::vector<int>* vt_filter_list_ = nullptr;
    const std::vector<int>* vt_filter_offsets_ = nullptr;

    CollisionInfo info_;

};

// topology filtering
inline bool vertex_adjacent_to_triangle(const VertexID v, const VertexID t1, const VertexID t2, const VertexID t3) {
    // Minimal: skip if v is on the triangle.
    return v == t1 || v == t2 || v == t3;
}

inline Vec3 closest_point_on_triangle(const Vec3& a,const Vec3& b,const Vec3& c,const Vec3& p) {
    const Vec3 ab = b-a;
    const Vec3 ac = c-a;
    // From "Real-Time Collision Detection" style barycentric region tests
    const Vec3 ap = p-a;
    const float d1 = ab.dot(ap);
    const float d2 = ac.dot(ap);
    if (d1 <= 0.f && d2 <= 0.f) return a;

    const Vec3 bp = p-b;
    const float d3 = ab.dot(bp);
    const float d4 = ac.dot(bp);
    if (d3 >= 0.f && d4 <= d3) return b;

    const float vc = d1*d4 - d3*d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        float v = d1 / (d1 - d3);
        return a + ab * v;
    }

    const Vec3 cp = p-c;
    const float d5 = ab.dot(cp);
    const float d6 = ac.dot(cp);
    if (d6 >= 0.f && d5 <= d6) return c;

    const float vb = d5*d2 - d1*d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }

    const float va = d3*d6 - d5*d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        const Vec3 bc = c-b;
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + bc * w;
    }

    const float denom = 1.f / (va + vb + vc);
    const float v = vb * denom;
    const float w = vc * denom;
    return a + ab * v + ac * w;
}

inline int binary_search_first_greater(const std::vector<int>& arr, int value, int begin, int end) {
    // returns first index i in [begin,end) s.t. arr[i] > value
    int l = begin, r = end;
    while (l < r) {
        int m = (l + r) >> 1;
        if (arr[m] <= value) l = m + 1;
        else r = m;
    }
    return l;
}





#endif //TAIYI_TRIMESHCOLLISION_H