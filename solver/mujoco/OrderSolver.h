//
// Created by tianyan on 2/26/26.
//

#ifndef TAIYI_ORDERSOLVER_H
#define TAIYI_ORDERSOLVER_H

#include "ISolver.h"
#include "Model.h"

class OrderSolver : public ISolver {
public:
    explicit OrderSolver(const MModel &model, const float angular_damping = 0.05f,
        const int update_mass_matrix_interval = 1, const float friction_smoothing = 1.0f) :
        model_(model),
        angular_damping(angular_damping),
        friction_smoothing(friction_smoothing),
        update_mass_matrix_interval(update_mass_matrix_interval) {};

    ~OrderSolver() override = default;

    void clear() override;

    void Step(State& state_in, State& state_out, const Contacts* contacts, float dt) override;

private:

    void compute_articulation_indices();

    void allocate_model_aux_vars();

    void allocate_state_aux_vars();

    void convert_body_force_com_to_origin(State& state_in) const;

    void eval_rigid_fk(State& state_in);

    void eval_rigid_id(const State& state_in);

private:

    const MModel&  model_;

    float angular_damping;

    float friction_smoothing;

    int update_mass_matrix_interval;

    // total flattened sizes
    int J_size = 0;
    int M_size = 0;
    int H_size = 0;

    // per-articulation offsets
    std::vector<int> articulation_J_start;
    std::vector<int> articulation_M_start;
    std::vector<int> articulation_H_start;

    // per-articulation matrix shapes
    std::vector<int> articulation_M_rows;   // = 6 * joint_count
    std::vector<int> articulation_H_rows;   // = dof_count
    std::vector<int> articulation_J_rows;   // = 6 * joint_count
    std::vector<int> articulation_J_cols;   // = dof_count

    // mapping to global joint-space arrays
    std::vector<int> articulation_dof_start;
    std::vector<int> articulation_coord_start;

    // system matrices (flattened)
    std::vector<float> M;
    std::vector<float> J;
    std::vector<float> P;
    std::vector<float> H;
    std::vector<float> L;

    //  model-dependent static caches
    std::vector<Mat66> body_I_m;
    std::vector<TTransform> body_X_com;

    // ---- Featherstone solver runtime aux vars ----
    std::vector<float> joint_qdd;
    std::vector<float> joint_tau;
    // std::vector<float> joint_solve_tmp;
    std::vector<SpatialVec> joint_S_s;          // per DOF motion subspace (6D)

    std::vector<TTransform> body_q_com;          // body COM pose in world
    std::vector<Mat66> body_I_s;                // body spatial inertia in current/world/solver frame
    std::vector<SpatialVec> body_v_s;           // body spatial velocity
    std::vector<SpatialVec> body_a_s;           // body spatial acceleration
    std::vector<SpatialVec> body_f_s;           // body spatial force (bias / intermediate)
    std::vector<SpatialVec> body_ft_s;          // body spatial force total / transformed

    bool state_aux_allocated_ = false;

    uint64_t topology_version_ = 0;

private:

    static Mat66 compute_spatial_inertia(const Mat3& I, float mass);

    static TTransform compute_com_transform(const Vec3& com);

    void compute_link_transform(int j, State& state_in);

    TTransform jcalc_transform(JointType type, int dof_start, int lin_axis_count, int ang_axis_count,
                               const std::vector<float> &joint_q, int q_start) const;

    SpatialVec jcalc_motion(JointType type, int lin_axis_count, int ang_axis_count, const TTransform &w_X_pj,
                            const std::vector<float> &joint_qd, int qd_start);

    static Quat quat_from_axis_angle(const Vec3& axis, float angle);

    void compute_link_velocity(int j, const State& state_in);

    static SpatialVec spatial_cross(const SpatialVec& a, const SpatialVec& b);

    static SpatialVec spatial_cross_dual(const SpatialVec& a, const SpatialVec& b);

    static Mat66 transform_spatial_inertia(const TTransform& w_X_cc, const Mat66& I_m);

    static SpatialVec transform_twist(const TTransform& X, const SpatialVec& twist_local);

};




#endif //TAIYI_ORDERSOLVER_H
