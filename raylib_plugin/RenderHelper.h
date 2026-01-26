//
// Created by 徐天焱 on 2025/11/5.
//

#ifndef TAIYI_RENDERHELPER_H
#define TAIYI_RENDERHELPER_H


#include <span>
#include <vector>
#include "raylib.h"
#include "Model.h" // MModel, State, MeshInfo, range, triangle
#include "Types.h" // Vec3, VertexID

class RenderHelper final {
public:
    RenderHelper() = default;
    ~RenderHelper() { Shutdown(); }

    RenderHelper(const RenderHelper&) = delete;
    RenderHelper& operator=(const RenderHelper&) = delete;

    void BindModel(const MModel& model);

    // rebuild if topology changed
    void Update(const State& state);

    static bool IsModelValid(const Model& model);
    static void UnloadRLModelSafe(Model& rl_model);

    void Draw(const State& state, bool is_wire_mode) const;
    void Shutdown();                 // release all gpu resources
    [[nodiscard]] bool Ready() const { return ready_; }

    Model& GetRLModel(size_t mesh_id);

private:
    struct RenderMesh {
        DeformableBodyInfo info{}; // value copy
        Model model{};   // raylib owning
        bool valid = false;
    };

    struct RenderRigidBody {
        RigidBodyInfo info{};
        Model model{};
        bool valid{false};
    };

private:
    void Rebuild();   // 用 model_ 重建所有 GPU mesh/model

    void UpdateDynamic(const State& state) const;

    static void FillPositionsXYZ(const State& state, size_t particle_begin, size_t particle_count, float* dst_xyz);

    static void ComputeNormalsXYZ(const MModel& model, std::span<const Vec3> pos, range tri_range, size_t particle_begin, size_t particle_count, float* dst_nxyz);

    static void BuildIndicesU16(const MModel& model,range tri_range, size_t particle_begin, size_t particle_count, unsigned short* dst_indices);

    static void QuatToAxisAngleDeg(const Quat& q_in, Vector3& axis, float& angle_deg);

private:
    const MModel* model_ = nullptr;

    uint64_t built_topology_version_ = 0;
    bool ready_ = false;

    std::vector<RenderMesh> meshes_;
    std::vector<RenderRigidBody> rigid_bodies_;
};




#endif //TAIYI_RENDERHELPER_H
