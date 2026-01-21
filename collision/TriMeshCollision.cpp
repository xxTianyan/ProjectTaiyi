//
// Created by tianyan on 1/20/26.
//

#include "TriMeshCollision.h"

#include "AdjacencyCSR.hpp"
#include "Model.h"

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
    Vec3 extend = cmax - cmin;
    int axis = 0;       // separate along x
    if (extend.y() > extend.x()) axis = 1;      // separate along y
    if ((axis==0 ? extend.x() : extend.y()) < extend.z()) axis = 2;     // separate along z

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

TriMeshCollisionDetector::TriMeshCollisionDetector(const MModel &model, const ForceElementAdjacencyInfo& adj, const int vertex_pre_alloc, const int vertex_max_alloc, const int edge_pre_alloc, const int edge_max_alloc, const int leaf_size)
    : model_(model), adj_(adj), v_pre_(vertex_pre_alloc), v_max_(vertex_max_alloc),
      e_pre_(edge_pre_alloc), e_max_(edge_max_alloc), leaf_size_(leaf_size) {

    const int V = static_cast<int>(model_.num_particles);
    const int T = static_cast<int>(model_.tris.size());
    const int E = static_cast<int>(model_.edges.size());

    tri_boxes_.resize(T);
    edge_boxes_.resize(E);

    // init buffer sizes / offsets
    v_buf_sizes_.assign(V, v_pre_);
    e_buf_sizes_.assign(E, e_pre_);
    compute_offsets(v_buf_sizes_, info_.vertex_colliding_triangles_offsets);
    compute_offsets(e_buf_sizes_, info_.edge_colliding_edges_offsets);

    // allocate flattened storage (2-int per record)
    info_.vertex_colliding_triangles.resize(2 * info_.vertex_colliding_triangles_offsets.back(), -1);
    info_.edge_colliding_edges.resize(2 * info_.edge_colliding_edges_offsets.back(), -1);

    info_.vertex_colliding_triangles_count.assign(V, 0);
    info_.vertex_colliding_triangles_min_dist.assign(V, 0.f);
    info_.triangle_colliding_vertices_min_dist.assign(T, 0.f);
    info_.edge_colliding_edges_count.assign(E, 0);
    info_.edge_colliding_edges_min_dist.assign(E, 0.f);

    pos_prev_collision_detection_.resize(V);
    particle_conservative_bounds_.resize(V);

    // build tree
    build(model_.particle_pos0);

}

void TriMeshCollisionDetector::set_vertex_triangle_filter_list(const std::vector<int> *list,
    const std::vector<int> *offsets) {
    vt_filter_list_ = list;
    vt_filter_offsets_ = offsets;
}

void TriMeshCollisionDetector::build(const std::vector<Vec3> &pos) {
    compute_tri_aabbs(pos);
    compute_edge_aabbs(pos);
    bvh_tris_.build(tri_boxes_, leaf_size_);
    bvh_edges_.build(edge_boxes_, leaf_size_);
}

void TriMeshCollisionDetector::refit(const std::vector<Vec3> &pos) {
    compute_tri_aabbs(pos);
    compute_edge_aabbs(pos);
    bvh_tris_.refit(tri_boxes_);
    bvh_edges_.refit(edge_boxes_);
}

void TriMeshCollisionDetector::compute_particle_conservative_bounds() {
    const size_t V = particle_conservative_bounds_.size();
    for (size_t v = 0; v < V; ++v) {
        float min_dist = std::min(particle_contact_margin, info_.vertex_colliding_triangles_min_dist[v]);

        // bound from neighbor triangles (incident faces)
        {
            const uint32_t fs = adj_.vertex_faces.begin(v);
            const uint32_t fe = adj_.vertex_faces.end  (v);
            for (uint32_t i = fs; i < fe; ++i) {
                const uint32_t packed = adj_.vertex_faces.incidents[i];
                const uint32_t tri_id = AdjacencyCSR::unpack_id(packed);
                min_dist = std::min(min_dist, info_.triangle_colliding_vertices_min_dist[tri_id]);
            }
        }

        // bound from neighbor edges (incident bending edges), currently dismiss
        /*{
            const uint32_t es = adj_.vertex_edges.begin(v);
            const uint32_t ee = adj_.vertex_edges.end  (v);
            for (uint32_t i = es; i < ee; ++i) {
                const uint32_t packed = adj_.vertex_edges.incidents[i];
                const uint32_t edge_id = AdjacencyCSR::unpack_id(packed);
                const uint32_t order   = AdjacencyCSR::unpack_order(packed);

                // Newton: only if vertex is actually on the edge segment endpoints (v1/v2)
                if (order == 2u || order == 3u) {
                    min_dist = std::min(min_dist, info_.edge_colliding_edges_min_dist[edge_id]);
                }
            }
        }*/

        particle_conservative_bounds_[v] = conservative_bound_relaxation * min_dist;
    }
}


void TriMeshCollisionDetector::vertex_triangle_collision_detection(const std::vector<Vec3>& pos,
                                                                   const float max_query_radius,
                                                                   const float rest_exclusion_radius,
                                                                   const std::vector<Vec3> *min_distance_filtering_ref_pos) {
    const int V = static_cast<int>(model_.num_particles);
    info_.vertex_overflow = 0;
    std::ranges::fill(info_.vertex_colliding_triangles, -1);
    std::ranges::fill(info_.triangle_colliding_vertices_min_dist, max_query_radius);


    for (int v = 0; v < V; ++v) {
        const Vec3& pv = pos[v];
        AABB q;
        q.lo = {pv.x() - max_query_radius, pv.y() - max_query_radius, pv.z() - max_query_radius};
        q.hi = {pv.x() + max_query_radius, pv.y() + max_query_radius, pv.z() + max_query_radius};

        int off = info_.vertex_colliding_triangles_offsets[v];
        int cap = info_.vertex_colliding_triangles_offsets[v+1] - off;

        int count = 0;
        float min_vertex_to_tri_dist = max_query_radius;

        bvh_tris_.query_aabb(q, [&](const int tri_id){
            const auto& tri = model_.tris[static_cast<size_t>(tri_id)].vertices;
            const auto t1 = tri[0], t2 = tri[1], t3 = tri[2];

            // 1) adjacency skip
            if (vertex_adjacent_to_triangle(static_cast<VertexID>(v), t1, t2, t3))
                return;

            // 2) optional per-vertex filtering list (sorted)
            if (vt_filter_list_ && vt_filter_offsets_) {
                const int fs = (*vt_filter_offsets_)[v];
                const int fe = (*vt_filter_offsets_)[v+1];
                if (fe > fs) {
                    const int first = (*vt_filter_list_)[fs];
                    const int last  = (*vt_filter_list_)[fe-1];
                    if (tri_id >= first && tri_id <= last) {
                        const int idx = binary_search_first_greater(*vt_filter_list_, tri_id, fs, fe);
                        if (idx > fs && (*vt_filter_list_)[idx-1] == tri_id)
                            return;
                    }
                }
            }

            // 3) narrow-phase distance at current pose
            const Vec3 &a = pos[t1], &b = pos[t2], &c = pos[t3];
            const Vec3 cp = closest_point_on_triangle(a,b,c,pv);
            const float dist = (cp - pv).norm();

            // 4) rest-shape exclusion
            if (min_distance_filtering_ref_pos && rest_exclusion_radius > 0.f) {
                const auto& ref = *min_distance_filtering_ref_pos;
                const Vec3 ar = ref[t1], br = ref[t2], cr = ref[t3], vr = ref[v];
                const Vec3 cpr = closest_point_on_triangle(ar, br, cr, vr);
                if (const float dist_ref = (cpr - vr).norm(); dist_ref < rest_exclusion_radius) return;
            }

            if (dist < max_query_radius) {
                min_vertex_to_tri_dist = std::min(min_vertex_to_tri_dist, dist);
                info_.triangle_colliding_vertices_min_dist[tri_id] =
                    std::min(info_.triangle_colliding_vertices_min_dist[tri_id], dist);
                if (count < cap) {
                    info_.vertex_colliding_triangles[2 * (off + count) + 0] = v;
                    info_.vertex_colliding_triangles[2 * (off + count) + 1] = tri_id;
                }else {
                    info_.vertex_overflow = 1;
                }
                ++count;
            }
        });

        info_.vertex_colliding_triangles_count[v] = count;
        info_.vertex_colliding_triangles_min_dist[v] = min_vertex_to_tri_dist;
    }
}

void TriMeshCollisionDetector::ensure_capacity_for_vertex_buffers() {
    if (!info_.vertex_overflow) return;
    // conservative strategy: double all, clamp to max
    for (auto& s : v_buf_sizes_) s = std::min(s * 2, v_max_);
    compute_offsets(v_buf_sizes_, info_.vertex_colliding_triangles_offsets);
    info_.vertex_colliding_triangles.assign(2 * info_.vertex_colliding_triangles_offsets.back(), -1);
    info_.vertex_overflow = 0;
}

void TriMeshCollisionDetector::collision_detection(const State &state_in) {
    refit(state_in.particle_pos);
    vertex_triangle_collision_detection(state_in.particle_pos, particle_contact_margin,
                                        particle_rest_shape_contact_exclusion_radius, &model_.particle_pos0);

    /*if (info_.vertex_overflow) {
        ensure_capacity_for_vertex_buffers();
        vertex_triangle_collision_detection(particle_contact_margin, particle_rest_shape_contact_exclusion_radius,
                                        &model_.particle_pos0);
    }*/

    // edge ...

    // init pos_prev_collision_detection
    pos_prev_collision_detection_ = state_in.particle_pos;
    compute_particle_conservative_bounds();
}

Vec3 TriMeshCollisionDetector::apply_conservative_bounds(const VertexID v, const Vec3 &inertia) const {
    const Vec3& prev_pos = pos_prev_collision_detection_[v];
    Vec3 disp = inertia - prev_pos;

    const float bound = particle_conservative_bounds_[v];

    const float disp_norm = disp.norm();
    if (disp_norm > bound && bound > 1e-5) {
        disp = disp * (bound / disp_norm);
        return prev_pos + disp;
    }
    return inertia;
}

void TriMeshCollisionDetector::compute_offsets(const std::vector<int> &sizes, std::vector<int> &offsets) {
    offsets.resize(sizes.size() + 1);
    offsets[0] = 0;
    for (size_t i = 0; i < sizes.size(); ++i)
        offsets[i+1] = offsets[i] + sizes[i];
}

void TriMeshCollisionDetector::compute_tri_aabbs(const std::vector<Vec3> &pos) {
    for (size_t i = 0; i < model_.tris.size(); ++i) {
        const auto& tv = model_.tris[i].vertices;
        AABB b;
        b.expand(pos[tv[0]]);
        b.expand(pos[tv[1]]);
        b.expand(pos[tv[2]]);
        tri_boxes_[i] = b;
    }
}

void TriMeshCollisionDetector::compute_edge_aabbs(const std::vector<Vec3> &pos) {
    for (size_t i = 0; i < model_.edges.size(); ++i) {
        // Newton: endpoints are vertices[2], vertices[3]
        VertexID v1 = model_.edges[i].vertices[2];
        VertexID v2 = model_.edges[i].vertices[3];
        AABB b;
        b.expand(pos[v1]);
        b.expand(pos[v2]);
        edge_boxes_[i] = b;
    }
}



