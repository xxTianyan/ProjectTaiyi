//
// Created by tianyan on 1/27/26.
//

#include "Collide.h"

#include "Model.h"

inline Vec3 ClosestPointPlane_Local(float sx, float sz, const Vec3& p_local) {
    const float x = std::max(-sx, std::min(sx, p_local.x()));
    const float z = std::max(-sz, std::min(sz, p_local.z()));
    return {x, 0.0f, z};
}

void CollisionPipeline::BroadphaseRigidPairs_(const MModel &model, const State &state) {
    if (!cached_contacts_) cached_contacts_ = std::make_unique<Contacts>();
    int out_count{0};

    const auto num_shapes = static_cast<int>(model.num_shapes);

    if (num_shapes <= 0) return;
    if (shape_pairs_filtered_.empty()) return;
    if (rigid_contact_max_ <= 0) return;

    for (auto [shape_a, shape_b] : shape_pairs_filtered_) {
        if (shape_a < 0 || shape_a >= num_shapes || shape_b < 0 || shape_b >= num_shapes) continue;
        if (shape_a == shape_b) continue;

        // --- build X_ws_a / X_ws_b (world space pos+rot) ---
        Vec3 p_ws_a, p_ws_b;
        Quat q_ws_a, q_ws_b;

        const int rigid_a = model.shape_body[shape_a];
        if (rigid_a < 0) {
            p_ws_a = model.shape_pos0[shape_a];
            q_ws_a = model.shape_rot0[shape_a];
        } else {
            // body * shape_local
            p_ws_a = state.body_pos[rigid_a] + state.body_rot[rigid_a] * model.shape_pos0[shape_a];
            q_ws_a = state.body_rot[rigid_a] * model.shape_rot0[shape_a];
        }

        const int rigid_b = model.shape_body[shape_b];
        if (rigid_b < 0) {
            p_ws_b = model.shape_pos0[shape_b];
            q_ws_b = model.shape_rot0[shape_b];
        } else {
            p_ws_b = state.body_pos[rigid_b] + state.body_rot[rigid_b] * model.shape_pos0[shape_b];
            q_ws_b = state.body_rot[rigid_b] * model.shape_rot0[shape_b];
        }

        int type_a = model.shape_type[shape_a];
        int type_b = model.shape_type[shape_b];

        if (type_a > type_b) {
            std::swap(shape_a, shape_b);
            std::swap(type_a, type_b);
            std::swap(p_ws_a, p_ws_b);
            std::swap(q_ws_a, q_ws_b);
        }

        const auto geo_a = static_cast<GeoType>(type_a);
        const auto geo_b = static_cast<GeoType>(type_b);

        // plane-plane: skip
        if (geo_a == GeoType::PLANE && geo_b == GeoType::PLANE) continue;

        // per-shape margin (sum)
        const float margin = model.shape_contact_margin[shape_a] + model.shape_contact_margin[shape_b];

        // bounding sphere check
        bool pass = false;
        if (geo_a == GeoType::PLANE) {
            // query_b = inv(X_ws_a) * p_b
            // local: p_local = R_a^T * (p_b - p_a)
            const Vec3 query_b = q_ws_a.conjugate() * (p_ws_b - p_ws_a);

            const Vec3 scale_a = model.shape_scale[shape_a];
            const Vec3 closest = ClosestPointPlane_Local(scale_a.x(), scale_a.z(), query_b);
            const float d = (query_b - closest).norm();

            const float r_b = model.shape_collision_radius[shape_b];
            pass = (d <= r_b + margin);
        } else {
            const float r_a = model.shape_collision_radius[shape_a];
            const float r_b = model.shape_collision_radius[shape_b];
            const float d = (p_ws_a - p_ws_b).norm();
            pass = (d <= r_a + r_b + margin);
        }

        if (!pass) continue;

        const int pair_index_ab = shape_a * num_shapes + shape_b;
        const int pair_index_ba = shape_b * num_shapes + shape_a;

        // --- count_contact_points_for_pair  ---
        int num_contacts_a = 0;
        int num_contacts_b = 0;
        if (geo_a == GeoType::PLANE) {
            // plane situation
            num_contacts_a = 0;
            num_contacts_b = ShapeContactPointCount(geo_b);
        } else {
            num_contacts_a = ShapeContactPointCount(geo_a);
            num_contacts_b = ShapeContactPointCount(geo_b);
        }

        // allocate contact points


        }

};

int CollisionPipeline::ShapeContactPointCount(const GeoType type) {
    switch (type) {
        case GeoType::PLANE:   return 0;
        case GeoType::SPHERE:  return 1;
        case GeoType::CAPSULE: return 2;
        case GeoType::BOX:     return 8;
        default:
            return 0;
    }
}