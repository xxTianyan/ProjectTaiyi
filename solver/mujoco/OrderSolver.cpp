//
// Created by tianyan on 2/26/26.
//

#include "OrderSolver.h"

void OrderSolver::clear() {
    if (topology_version_ != model_.topology_version) {
        topology_version_ = model_.topology_version;
        compute_articulation_indices();
        allocate_model_aux_vars();
    }
}

void OrderSolver::Step(State &state_in, State &state_out, const Contacts *contacts, float dt) {


}

void OrderSolver::compute_articulation_indices() {
    // calculate total size and offsets of Jacobian and mass matrices for entire system
    J_size = 0;
    M_size = 0;
    H_size = 0;

    articulation_J_start.clear();
    articulation_M_start.clear();
    articulation_H_start.clear();

    articulation_M_rows.clear();
    articulation_H_rows.clear();
    articulation_J_rows.clear();
    articulation_J_cols.clear();

    articulation_dof_start.clear();
    articulation_coord_start.clear();

    if (model_.num_joints == 0)
        return;

    if (model_.articulation_start.size() != model_.num_articulation + 1) {
        throw std::runtime_error(
            "compute_articulation_indices: model_.articulation_start must contain a sentinel "
            "(size must be num_articulation + 1).");
    }

    if (model_.joint_q_start.size() != model_.num_joints + 1) {
        throw std::runtime_error(
            "compute_articulation_indices: model_.joint_q_start must contain a sentinel "
            "(size must be num_joints + 1).");
    }

    if (model_.joint_qd_start.size() != model_.num_joints + 1) {
        throw std::runtime_error(
            "compute_articulation_indices: model_.joint_qd_start must contain a sentinel "
            "(size must be num_joints + 1).");
    }

    // Reserve to avoid reallocs
    articulation_J_start.reserve(model_.num_articulation);
    articulation_M_start.reserve(model_.num_articulation);
    articulation_H_start.reserve(model_.num_articulation);

    articulation_M_rows.reserve(model_.num_articulation);
    articulation_H_rows.reserve(model_.num_articulation);
    articulation_J_rows.reserve(model_.num_articulation);
    articulation_J_cols.reserve(model_.num_articulation);

    articulation_dof_start.reserve(model_.num_articulation);
    articulation_coord_start.reserve(model_.num_articulation);

    // -------- main loop --------
    for (size_t i = 0; i < model_.num_articulation; ++i) {
        const int first_joint = model_.articulation_start[i];
        const int last_joint  = model_.articulation_start[i + 1];   // sentinel-style next start

        // q/qd start of the first joint in this articulation
        const int first_coord = model_.joint_q_start[first_joint];
        const int first_dof   = model_.joint_qd_start[first_joint];

        // qd start of the first joint AFTER this articulation
        // (because joint_qd_start has sentinel, this is safe even if last_joint == num_joints)
        const int last_dof    = model_.joint_qd_start[last_joint];

        const int joint_count_local = last_joint - first_joint;
        const int dof_count_local   = last_dof - first_dof;

        if (joint_count_local < 0 || dof_count_local < 0) {
            throw std::runtime_error(
                "compute_articulation_indices: invalid articulation layout (negative joint/dof count).");
        }

        // record flattened offsets
        articulation_J_start.push_back(J_size);
        articulation_M_start.push_back(M_size);
        articulation_H_start.push_back(H_size);

        articulation_dof_start.push_back(first_dof);
        articulation_coord_start.push_back(first_coord);

        // record matrix shapes for this articulation
        // J: (6 * joint_count) x dof_count
        // M: (6 * joint_count) x (6 * joint_count)
        // H: dof_count x dof_count
        const int spatial_rows = 6 * joint_count_local;

        articulation_M_rows.push_back(spatial_rows);
        articulation_H_rows.push_back(dof_count_local);
        articulation_J_rows.push_back(spatial_rows);
        articulation_J_cols.push_back(dof_count_local);

        // total flattened storage sizes
        J_size += spatial_rows * dof_count_local;
        M_size += spatial_rows * spatial_rows;
        H_size += dof_count_local * dof_count_local;
    }
}

void OrderSolver::allocate_model_aux_vars() {

    // Allocate system matrices
    if (model_.num_joints > 0) {
        M.assign(M_size, 0.0f);
        J.assign(J_size, 0.0f);
        P.resize(J_size);
        H.resize(H_size);
        L.assign(H_size, 0.0f);   // zero because later factorization may only write one triangle
    } else {
        M.clear();
        J.clear();
        P.clear();
        H.clear();
        L.clear();
    }

    // precompute static per-body solver data
    if (model_.num_bodies > 0) {
        body_I_m.resize(model_.num_bodies);
        body_X_com.resize(model_.num_bodies);

        for (size_t b = 0; b < model_.num_bodies; ++b) {
            body_I_m[b] = compute_spatial_inertia(model_.body_inertia[b], model_.body_inv_mass[b]);
            body_X_com[b] = compute_com_transform(model_.body_local_com[b]);
        }
    } else {
        body_I_m.clear();
        body_X_com.clear();
    }

}

Mat66 OrderSolver::compute_spatial_inertia(const Mat3 &I, const float mass) {
    Mat66 out = Mat66::Zero();
    // top-left = m * I3
    out.block<3,3>(0,0) = 1 / mass * Mat3::Identity();

    // bottom-right = rotational inertia
    out.block<3,3>(3,3) = I;

    return out;
}

TTransform OrderSolver::compute_com_transform(const Vec3 &com) {
    TTransform X;
    X.p = com;
    X.q = Quat::Identity();
    return X;

};

