//
// Created by tianyan on 1/27/26.
//

#ifndef TAIYI_COLLIDE_H
#define TAIYI_COLLIDE_H

#include <memory>
#include <vector>

#include "Types.h"

struct GeoData;
enum class GeoType;
struct MModel;
struct State;
struct Contacts;

struct CollideParams {
    // soft
    float soft_contact_margin = 0.01f;
    int   edge_sdf_iter       = 10;

    // capacity controls,  -1 means None
    int rigid_contact_max          = 1000;
    int rigid_contact_max_per_pair = -1;
    int soft_contact_max           = -1;

    // debug / perf
    bool rebuild_pipeline_if_needed = false;
};

struct ContactManifold {
    Vec3 p_a_world;
    Vec3 p_b_world;
    Vec3 normal;
    float distance;
};

class CollisionPipeline {
public:
    CollisionPipeline() = default;
    ~CollisionPipeline() = default;

    CollisionPipeline(const CollisionPipeline&) = delete;
    CollisionPipeline& operator=(const CollisionPipeline&) = delete;

    // Build pipeline from model (allocate buffers, copy filtered pairs, etc.)
    // make contact ptr in this function.
    void BuildFromModel(const MModel& model, const CollideParams& params);

    // Update per-call parameters (Newton: self.soft_contact_margin, self.edge_sdf_iter)
    void SetSoftContactMargin(const float m) { soft_contact_margin_ = m; }
    void SetEdgeSdfIter(const int iters)     { edge_sdf_iter_ = std::max(0, iters); }

    [[nodiscard]] int ShapeCount()    const { return shape_count_; }
    [[nodiscard]] int ParticleCount() const { return particle_count_; }

    [[nodiscard]] int RigidContactMax() const { return rigid_contact_max_; }
    [[nodiscard]] int SoftContactMax()  const { return soft_contact_max_;  }

    /* internal cached contacts
     * NOTE:
     * 1. need topology version check
     * 2. need to clear contact first
     */
    Contacts& Collide(const MModel& model, const State& state);

private:

    // Pipeline stages
    void GenerateSoftContacts_(const MModel& model, const State& state);
    void BroadPhaseRigidPairs_(const MModel& model, const State& state);
    void NarrowPhaseRigidContacts_(const MModel& model, const State& state);

    static int ShapeContactPointCount(GeoType type);

    // collision
    static ContactManifold box_plane_collision(const GeoData &box, const GeoData &plane, int point_id, int edge_sdf_iter);

private:
    bool AllocateContactPoints_(int num_contacts_a, int num_contacts_b, int shape_a, int shape_b);

    static std::pair<int, int> CountContactPointsForPair_(const std::vector<Vec3> &shape_scale, int shape_a, int shape_b, GeoType type_a, GeoType type_b);

    static Vec3 get_box_vertex(int point_id, const Vec3& upper);
private:
    //contact point pair buffer count
    int rigid_pair_count_{0};

    // Immutable-ish after build
    int shape_count_{0};
    int particle_count_{0};

    // shape_contact_pair from model
    std::vector<std::pair<int, int>> shape_contact_pair;

    // Capacity strategy
    int rigid_contact_max_{0};
    int rigid_contact_max_per_pair_{0}; // 0 means no limit
    int soft_contact_max_{0};

    // Dynamic per-call params
    float soft_contact_margin_{0.01f};
    int edge_sdf_iter_{10};

    // broad phase pair buffers, length = rigid_contact_max_
    std::vector<int> rigid_pair_shape0_;
    std::vector<int> rigid_pair_shape1_;
    std::vector<int> rigid_pair_point_id_;

    // per-pair control (currently not use)
    // std::vector<int> rigid_pair_point_limit_; // now empty
    // std::vector<int> rigid_pair_point_count_; // now empty

    // Cached contacts
    std::unique_ptr<Contacts> cached_contacts_;

    uint64_t topology_version = 0;
};

#endif //TAIYI_COLLIDE_H