//
// Created by tianyan on 1/27/26.
//

#include "Collide.h"
#include "Geometry.h"
#include "Model.h"

inline Vec3 ClosestPointPlane_Local(const float sx, const float sz, const Vec3& p_local) {
    float x, z;
    if (sx > 0.0f)
        x = std::max(-sx, std::min(sx, p_local.x()));
    else
        x = p_local.x();

    if (sz > 0.0f)
        z = std::max(-sz, std::min(sz, p_local.z()));
    else
        z = p_local.z();

    return {x, 0.0f, z};
}

void CollisionPipeline::BuildFromModel(const MModel &model, const CollideParams &params) {

    shape_count_ = model.num_shapes;
    particle_count_ = model.num_particles;

    const size_t num_shape_pair = model.shape_contact_pairs.size();

    shape_contact_pair.assign(model.shape_contact_pairs.begin(), model.shape_contact_pairs.end());

    // rigid capacity
    if (params.rigid_contact_max_per_pair > 0)
        rigid_contact_max_per_pair_ = params.rigid_contact_max_per_pair;
    else
        rigid_contact_max_per_pair_ = 0;

    if (params.rigid_contact_max > 0)
        rigid_contact_max_ = params.rigid_contact_max;
    else
        rigid_contact_max_ = rigid_contact_max_per_pair_ * num_shape_pair;

    if (rigid_contact_max_ <= 0)
        throw std::runtime_error("Collide: rigid_contact_max_ incorrect setting.");

    // soft capacity
    if (params.soft_contact_max <= 0) {
        long long prod = 1LL * shape_count_ * particle_count_;
        if (prod > static_cast<long long>(std::numeric_limits<int>::max()))
            prod = static_cast<long long>(std::numeric_limits<int>::max());
        soft_contact_max_ = static_cast<int>(prod);
    }else
        soft_contact_max_ = params.soft_contact_max;

    soft_contact_margin_ = params.soft_contact_margin;
    edge_sdf_iter_ = params.edge_sdf_iter;

    rigid_pair_shape0_.assign(rigid_contact_max_, -1);
    rigid_pair_shape1_.assign(rigid_contact_max_, -1);
    rigid_pair_point_id_.assign(rigid_contact_max_, -1);

    // create contacts
    if (cached_contacts_ == nullptr)
        cached_contacts_ = std::make_unique<Contacts>(rigid_contact_max_, soft_contact_max_);

    topology_version = model.topology_version;
}

Contacts& CollisionPipeline::Collide(const MModel &model, const State &state) {

    const bool need_rebuild = (topology_version != model.topology_version) || model.collide_params.rebuild_pipeline_if_needed;
    if (need_rebuild)
        BuildFromModel(model, model.collide_params);

    cached_contacts_->clear();

    // GenerateSoftContacts_(model, state);
    BroadPhaseRigidPairs_(model, state);
    // NarrowPhaseRigidContacts_(model, state);
    return *cached_contacts_;
}

void CollisionPipeline::BroadPhaseRigidPairs_(const MModel &model, const State &state) {
    if (!cached_contacts_) cached_contacts_ = std::make_unique<Contacts>();
    const auto num_shapes = static_cast<int>(model.num_shapes);

    // boundary check
    if (num_shapes <= 0) return;
    if (shape_contact_pair.empty()) return;
    if (rigid_contact_max_ <= 0) return;

    // reset this frame's pair list
    rigid_pair_count_ = 0;

    for (auto [shape_a, shape_b] : shape_contact_pair) {
        if (shape_a < 0 || shape_a >= num_shapes || shape_b < 0 || shape_b >= num_shapes)
            throw std::runtime_error("Invalid shape pair appear");

        if (shape_a == shape_b) continue;

        // --- build X_ws_a / X_ws_b (shape -> world pos+rot) ---
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

        // ensure unique ordering of shape pairs
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
            // rotate to plane coordinate
            const Vec3 query_b = q_ws_a.conjugate() * (p_ws_b - p_ws_a);
            const Vec3& scale_a = model.shape_scale[shape_a];
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

        // flatten 2d array
        /*const int pair_index_ab = shape_a * num_shapes + shape_b;
        const int pair_index_ba = shape_b * num_shapes + shape_a;*/

        // --- count_contact_points_for_pair  ---
        auto [num_contacts_a, num_contacts_b] = CountContactPointsForPair_(
            model.shape_scale, shape_a, shape_b, static_cast<GeoType>(type_a), static_cast<GeoType>(type_b));

        // assign a limit contacts num for each rigid contact pair, if flag is on, currently useless
        /*if (rigid_contact_max_per_pair_ > 0) {
            const int half = rigid_contact_max_per_pair_ / 2;
            if (num_contacts_b > 0) {
                rigid_pair_point_limit_[pair_index_ab] = half;
                rigid_pair_point_limit_[pair_index_ba] = half;
            } else {
                rigid_pair_point_limit_[pair_index_ab] = rigid_contact_max_per_pair_;
                rigid_pair_point_limit_[pair_index_ba] = 0;
            }
        } else {
            rigid_pair_point_limit_[pair_index_ab] = 0;
            rigid_pair_point_limit_[pair_index_ba] = 0;
        }*/

        // allocate contact points
        const bool _success = AllocateContactPoints_(num_contacts_a, num_contacts_b, shape_a, shape_b);
        if (!_success)
            break;
    }
}

void CollisionPipeline::NarrowPhaseRigidContacts_(const MModel &model, const State &state) {
    if (cached_contacts_ == nullptr) return;

    // 1) narrow-phase produces "packed" contacts (like Newton's counter_increment)
    int& out_count = cached_contacts_->rigid_contact_count();
    out_count = 0;

    // 2) only iterate the valid pair buffer range for this frame
    //    (broad phase must have set rigid_pair_count_)
    const int pair_count = rigid_pair_count_;
    if (pair_count <= 0) return;

    for (int i = 0; i < pair_count; i++) {
        const int shape_a = rigid_pair_shape0_[i];
        const int shape_b = rigid_pair_shape1_[i];

        if (shape_a < 0 || shape_b < 0) continue;
        if (shape_a == shape_b) continue;

        if (rigid_contact_max_per_pair_ > 0) {
            // pairwise limit, currently empty
        }

        const GeoData geo_a = GeoData::CreateGeoData(shape_a, model, state);
        const GeoData geo_b = GeoData::CreateGeoData(shape_b, model, state);

        const float rigid_contact_margin = model.shape_contact_margin[shape_a] + model.shape_contact_margin[shape_b];

        Vec3 pA_w = Vec3::Zero();
        Vec3 pB_w = Vec3::Zero();
        Vec3 n_w = Vec3::UnitY();
        float distance = 1e6;
        bool valid = false;

        // ----- narrow phase dispatch -------
        // IMPORTANT: normal must always point from A -> B, and distance should satisfy:

        if (geo_a.geo_type == GeoType::PLANE && geo_b.geo_type == GeoType::BOX) {
            valid = plane_box_collision(geo_a, geo_b, pA_w, pB_w, n_w, distance);
            if (valid) {
                Vec3 p_plane, p_box, n;
                float dist = 1.0e6f;
                valid = plane_box_collision(geo_a, geo_b, p_plane, p_box, n, dist);
            }

        } else
            continue;

        if (!valid) continue;

        // skin/thickness logic matching your kernel
        const float thickness_pair = geo_a.thickness + geo_b.thickness;
        const float total_separation_needed = geo_a.radius_eff + geo_b.radius_eff + thickness_pair;

        const float d = distance - total_separation_needed;
        if (d >= rigid_contact_margin) continue;

        // write outputs
        cached_contacts_->rigid_contact_shape0[i] = shape_a;
        cached_contacts_->rigid_contact_shape1[i] = shape_b;

        // store body-local points (NOT shape-local): matches your wp.transform_point(geo.X_bw, p_world)
        cached_contacts_->rigid_contact_point0[i] = geo_a.X_bw.transformPoint(pA_w);
        cached_contacts_->rigid_contact_point1[i] = geo_b.X_bw.transformPoint(pB_w);

        const float offset_mag_a = geo_a.radius_eff + geo_a.thickness;
        const float offset_mag_b = geo_b.radius_eff + geo_b.thickness;

        cached_contacts_->rigid_contact_normal[i] = n_w;
        cached_contacts_->rigid_contact_thickness0[i] = offset_mag_a;
        cached_contacts_->rigid_contact_thickness1[i] = offset_mag_b;
    }
};

bool CollisionPipeline::AllocateContactPoints_(const int num_contacts_a, const int num_contacts_b, const int shape_a, const int shape_b) {

    const int num_contacts = num_contacts_a + num_contacts_b;
    if (num_contacts <= 0) return true;

    const int start = rigid_pair_count_;
    const int end_exclusive = start + num_contacts;

    if (end_exclusive > rigid_contact_max_) return false;

    // A -> B
    for (int i = 0; i < num_contacts_a; ++i) {
        const int idx = start + i;
        rigid_pair_shape0_[idx]   = shape_a;
        rigid_pair_shape1_[idx]   = shape_b;
        rigid_pair_point_id_[idx] = i;
    }

    // B -> A
    for (int i = 0; i < num_contacts_b; ++i) {
        const int idx = start + num_contacts_a + i;
        rigid_pair_shape0_[idx]   = shape_b;
        rigid_pair_shape1_[idx]   = shape_a;
        rigid_pair_point_id_[idx] = i;
    }

    rigid_pair_count_ = end_exclusive;

    return true;
}

std::pair<int, int> CollisionPipeline::CountContactPointsForPair_(const std::vector<Vec3> &shape_scale,
                                                                  const int shape_a, int shape_b, const GeoType type_a,
                                                                  const GeoType type_b) {

    // --- PLANE vs all ---
    if (type_a == GeoType::PLANE) {
        const Vec3& scale_a = shape_scale[shape_a];
        const bool infinite_plane = (scale_a.x() == 0.0f && scale_a.z() == 0.0f);

        if (type_b == GeoType::PLANE) return {0, 0};
        if (type_b == GeoType::SPHERE) return {1, 0};

        if (type_b == GeoType::CAPSULE) {
            // 2 endpoints (+4 plane edges if finite)
            return infinite_plane ? std::make_pair(2, 0) : std::make_pair(2 + 4, 0);
        }

        if (type_b == GeoType::CYLINDER) {
            // Newton: infinite plane supports 4 primitive contacts (2 caps + 2 side)
            return {4, 0};
        }

        if (type_b == GeoType::BOX) {
            // 8 vertices (+4 plane edges if finite)
            return infinite_plane ? std::make_pair(8, 0) : std::make_pair(8 + 4, 0);
        }
    }

    return std::make_pair(0, 0);
}

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

bool CollisionPipeline::plane_box_collision(const GeoData &plane, const GeoData &box, Vec3 &p_plane_w, Vec3 &p_box_w,
    Vec3 &n_w, float &distance) {


}
