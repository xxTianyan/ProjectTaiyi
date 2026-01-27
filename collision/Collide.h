//
// Created by tianyan on 1/27/26.
//

#ifndef TAIYI_COLLIDE_H
#define TAIYI_COLLIDE_H
#include <memory>
#include <vector>

struct MModel;
struct State;
struct Contacts;

struct ShapePair {
    int s0{-1};
    int s1{-1};
};

struct CollideParams {
    // soft
    float soft_contact_margin = 0.01f;
    int   edge_sdf_iter       = 10;

    // capacity controls (可选：让 pipeline 预估或固定上限)
    int rigid_contact_max_per_pair = -1; // -1 表示“不限制/由 pipeline 自己策略”
    int soft_contact_max           = -1;

    // debug / perf
    bool rebuild_pipeline_if_needed = true;
};

class CollisionPipeline {
public:
    CollisionPipeline() = default;
    ~CollisionPipeline() = default;

    CollisionPipeline(const CollisionPipeline&) = delete;
    CollisionPipeline& operator=(const CollisionPipeline&) = delete;

    // Build pipeline from model (allocate buffers, copy filtered pairs, etc.)
    // 对齐 Newton: CollisionPipeline.from_model(...)
    void BuildFromModel(const MModel& model, const CollideParams& params);

    // Ensure internal buffers match model (shape count changed / filtered pairs changed / topology version changed)
    void EnsureUpToDate(const MModel& model, const CollideParams& params);

    // Update per-call parameters (Newton: self.soft_contact_margin, self.edge_sdf_iter)
    void SetSoftContactMargin(float m) { soft_contact_margin_ = m; }
    void SetEdgeSdfIter(int iters)     { edge_sdf_iter_ = std::max(0, iters); }

    [[nodiscard]] int ShapeCount()    const { return shape_count_; }
    [[nodiscard]] int ParticleCount() const { return particle_count_; }

    [[nodiscard]] int RigidContactMax() const { return rigid_contact_max_; }
    [[nodiscard]] int SoftContactMax()  const { return soft_contact_max_;  }

    // internal cached contacts (Newton: self.contacts)
    // 如果你更喜欢 Newton 方式：Collide() 返回引用/指针，也可以打开这个。
    Contacts& CollideCached(const MModel& model, const State& state);

private:
    // --------------------------
    // Internal buffer management
    // --------------------------
    void AllocateOrResize_(const CollideParams& params);
    void ClearPairBuffers_();
    void ClearCachedContacts_();

    // --------------------------
    // Pipeline stages (CPU版可以直接实现; GPU版对应 kernel launch)
    // --------------------------
    void GenerateSoftContacts_(const MModel& model, const State& state, Contacts& contacts);
    void BroadphaseRigidPairs_(const MModel& model, const State& state);
    void NarrowphaseRigidContacts_(const MModel& model, const State& state, Contacts& contacts);

private:
    // --------------------------
    // Immutable-ish after build
    // --------------------------
    int shape_count_{0};
    int particle_count_{0};

    // Newton: shape_pairs_filtered (候选 shape pairs 列表)
    std::vector<ShapePair> shape_pairs_filtered_;
    int shape_pairs_max_{0};

    // Capacity strategy (Newton: rigid_contact_max / rigid_contact_max_per_pair / soft_contact_max)
    int rigid_contact_max_{0};
    int rigid_contact_max_per_pair_{0}; // 0 表示“不限制 per pair”，贴 Newton
    int soft_contact_max_{0};

    // Dynamic per-call params
    float soft_contact_margin_{0.01f};
    int edge_sdf_iter_{10};

    // --------------------------
    // Broadphase pair buffers (Newton: rigid_pair_*)
    // --------------------------
    // 长度 = rigid_contact_max_
    std::vector<int> rigid_pair_shape0_;
    std::vector<int> rigid_pair_shape1_;
    std::vector<int> rigid_pair_point_id_;

    // Optional per-pair control (Newton has rigid_pair_point_limit/point_count, can be None)
    // 如果你先不做 manifold/每pair上限，可以先不启用它们
    std::vector<int> rigid_pair_point_limit_; // 可为空
    std::vector<int> rigid_pair_point_count_; // 可为空

    // --------------------------
    // Cached contacts (Newton: self.contacts)
    // --------------------------
    std::unique_ptr<Contacts> cached_contacts_;
    bool cache_enabled_{true};
};

#endif //TAIYI_COLLIDE_H