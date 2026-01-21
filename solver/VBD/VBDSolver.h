//
// Created by 徐天焱 on 2025/11/11.
//

#ifndef TAIYI_VBDDYNAMICS_H
#define TAIYI_VBDDYNAMICS_H

#include <span>
#include <Types.h>
#include "AdjacencyCSR.hpp"
#include "Scene.h"
#include "ISolver.h"
#include "MaterialParams.hpp"
#include "TriMeshCollision.h"

class SolverDebugger;

class VBDSolver final : public ISolver {

public:
    explicit VBDSolver(const MModel& model, const int num_iters, const MMaterial& material = default_cloth(), SolverDebugger* dbg = nullptr)
        : model_(model), num_iters(num_iters), dbg_(dbg),material_(material) {}
    ~VBDSolver() override = default;

    void Init() override;

    void Step(State& state_in, State& state_out, float dt) override;

    void forward_step(State& state_in, float dt);

    void forward_step_with_penetration(State& state_in, float dt);

    void solve(State& state_in, State& state_out, float dt) const;

    void update_velocity(State& stat_out, float dt) const;

    void set_self_collision(float particle_contact_margin, float particle_rest_shape_contact_exclusion_radius,
                            float conservative_bound_relaxation);

    static void accumulate_stvk_triangle_force_hessian(std::span<const Vec3> pos, const MMaterial& mat,
                                                       const triangle& face, uint32_t vtex_order, Vec3& force, Mat3& H);

    static void accumulate_dihedral_angle_based_bending_force_hessian(std::span<const Vec3> pos, const MMaterial& mat,
        const edge& e, uint32_t vtex_order, Vec3& force, Mat3& H) ;

    void accumulate_neo_hookean_tetrahedron_force_hessian(std::span<const Vec3> pos, const MMaterial& mat,
        const tetrahedron& tet, uint32_t vtex_order, Vec3& force, Mat3& H, /*for debug*/ size_t tet_id) const;

    void evaluate_static_plane_particle_contact(const Vec3 &x, const Vec3 &x_prev, const Vec3 &plane_point,
                                                const Vec3 &plane_n_unit,  float radius,  float ke,
                                                float kd_ratio, float friction_mu, float friction_epsilon, float dt,Vec3& f_out, Mat3& H_out) const;

    // detector relevant
    void draw_triangle_bvh(const AABBTreeDrawSettings& s) const {
        if (detector_)
            detector_->draw_triangle_bvh(s);
    };

private:

    void BuildAdjacencyInfo();

    const MModel&  model_;

    int num_iters;

    SolverDebugger* dbg_;

    std::unique_ptr<TriMeshCollisionDetector> detector_;

    MMaterial material_;

    std::vector<Vec3> inertia_;
    std::vector<Vec3> prev_pos_;

    ForceElementAdjacencyInfo adjacency_info_;

    // temporary
    std::vector<char> surface_vertices;

    uint64_t topology_version_ = 0;
};





#endif //TAIYI_VBDDYNAMICS_H
