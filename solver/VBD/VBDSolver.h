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

    void Step(State& state_in, State& state_out, const Contacts* contacts, float dt) override;

    void set_self_collision(float particle_contact_margin, float particle_rest_shape_contact_exclusion_radius,
                            float conservative_bound_relaxation);

    // detector relevant, temporary
    void draw_triangle_bvh(const AABBTreeDrawSettings& s) const {
        if (detector_)
            detector_->draw_triangle_bvh(s);
    };

    void set_num_pre_alloc_contacts(const int n_pre_alloc_contacts) {
        num_pre_alloc_contacts = n_pre_alloc_contacts;
    }

public:
    struct RigidContactEvalResult {
        Vec3 force_0, torque_0;
        Mat3 h_ll_0, h_al_0, h_aa_0;
        Vec3 force_1, torque_1;
        Mat3 h_ll_1, h_al_1, h_aa_1;
    };

private:

     void clear() override;

    // ------ particle pipeline ---------

    void init_particles(State& state_in, float dt);

    void forward_step(State& state_in, float dt);

    void forward_step_with_penetration(State& state_in, float dt);

    void solve_particle(State& state_in, State& state_out, float dt);

    static void accumulate_stvk_triangle_force_hessian(std::span<const Vec3> pos, const MMaterial &mat,
                                                       const triangle &face, uint32_t vtex_order, Vec3 &force, Mat3 &H);

    static void accumulate_dihedral_angle_based_bending_force_hessian(std::span<const Vec3> pos, const MMaterial &mat,
                                                                      const edge &e, uint32_t vtex_order, Vec3 &force,
                                                                      Mat3 &H);

    void accumulate_neo_hookean_tetrahedron_force_hessian(std::span<const Vec3> pos, const MMaterial &mat,
                                                          const tetrahedron &tet, uint32_t vtex_order, Vec3 &force,
                                                          Mat3 &H, /*for debug*/ size_t tet_id) const;

    void evaluate_static_plane_particle_contact(const Vec3 &x, const Vec3 &x_prev, const Vec3 &plane_point,
                                                const Vec3 &plane_n_unit,  float radius,  float ke,
                                                float kd_ratio, float friction_mu, float friction_epsilon, float dt,Vec3& f_out, Mat3& H_out) const;

    void evaluate_vertex_triangle_contact(VertexID v, std::span<const Vec3> pos, const triangle &face,
                                          float collision_radius, float collision_stiffness, float collision_damping,
                                          float friction_mu, float friction_epsilon, float dt);

    void update_particle_vel(State& stat_out, float dt);

    void BuildAdjacencyInfo();


    // ------- rigid body pipeline -------
    void init_rigid_bodies(State& state_in, const Contacts* contacts, float dt);

    void forward_step_rigid_bodies(State& state_in, float dt);

    static void integrate_rigid_body(const Vec3 &x0, const Quat &r0, const Vec3 &v0, const Vec3 &w0,
                                       const Vec3 &f_ext, const Vec3 &t_ext, const Vec3 &com_local, float inv_mass,
                                       const Mat3 &I_body, const Mat3 &inv_I_body, const Vec3 &gravity,
                                       float angular_damping, float dt, /*Outputs*/ Vec3 &x_out, Quat &r_out,
                                       Vec3 &v_out, Vec3 &w_out);

    void build_body_body_contact_lists(const Contacts* contacts);

    void warm_start_body_body_contact(const Contacts* contacts);

    void solve_rigid_body(const State& state_in, State& state_out, const Contacts* contacts, float dt);

    void accumulate_rigid_body_force_hessian(size_t body_idx, size_t cp_idx, const State& state_in, const Contacts* contacts, float dt);

    RigidContactEvalResult evaluate_rigid_contact_from_collision(int body0, int body1, const Vec3 &body0_pos, const Vec3 &body1_pos,
                                                                const Quat &body0_q, const Quat &body1_q,
                                                                const Vec3 &contact_point_a_local,
                                                                const Vec3 &contact_point_b_local,
                                                                const Vec3 &contact_normal, float penetration_depth,
                                                                float contact_ke, float contact_kd, float friction_mu,
                                                                float friction_epsilon, float dt);

    void compute_projected_isotropic_friction(float friction_mu, float normal_load, const Vec3 &n_unit,
                                             const Vec3 &slip_u, float eps_u, Vec3 &force_out, Mat3 &H_out);

    // ------ SOA Data -------

    const MModel&  model_;

    int num_iters;

    SolverDebugger* dbg_;

    std::unique_ptr<TriMeshCollisionDetector> detector_;

    MMaterial material_;

    //  ---- particle ----
    std::vector<Vec3> particle_inertia_;
    std::vector<Vec3> particle_prev_pos_;
    std::vector<Vec3> particle_contact_force_;
    std::vector<Mat3> particle_contact_hessian_;

    ForceElementAdjacencyInfo adjacency_info_;

    // temporary
    std::vector<char> surface_vertices;

    // ---- rigid body ----
    std::vector<Vec3> body_prev_pos_;
    std::vector<Quat> body_prev_rot_;
    std::vector<Vec3> body_inertia_pos_;
    std::vector<Quat> body_inertia_rot_;

    std::vector<Vec3> body_force_;
    std::vector<Vec3> body_torque_;
    std::vector<Mat3> body_hessian_aa_;
    std::vector<Mat3> body_hessian_al_;
    std::vector<Mat3> body_hessian_ll_;

    // ----- contact related ------
    std::vector<float> body_body_contact_penalty_k_;
    std::vector<float> body_body_contact_material_ke_;
    std::vector<float> body_body_contact_material_kd_;
    std::vector<float> body_body_contact_material_mu_;
    std::vector<size_t> body_body_contact_counts_;
    std::vector<size_t> body_body_contact_counts_indices_;
    size_t num_pre_alloc_contacts = 24;
    float k_start_body_contact = 100.0f;

    uint64_t topology_version_ = 0;
};

#endif //TAIYI_VBDDYNAMICS_H
