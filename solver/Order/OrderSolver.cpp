//
// Created by tianyan on 2/26/26.
//

#include "OrderSolver.h"
#include "Math.hpp"

void OrderSolver::clear() {
    if (topology_version_ != model_.topology_version) {
        topology_version_ = model_.topology_version;
        compute_articulation_indices();
        allocate_model_aux_vars();
    }
}

void OrderSolver::Step(State &state_in, State &state_out, const Contacts *contacts, float dt) {

    clear();

    if (!state_aux_allocated_)
        allocate_state_aux_vars();

    if (model_.num_bodies > 0)
        convert_body_force_com_to_origin(state_in);

    if (model_.num_joints > 0) {
        eval_rigid_fk(state_in);

         // evaluate joint inertias, motion vectors, and forces
        for (auto& f : body_f_s) { f.setZero();}

        eval_rigid_id(state_in);

        eval_body_contact(state_in, contacts);

        for (auto& f : body_ft_s) { f.setZero();}
        for (auto& t : joint_tau) { t = 0.0f;}

        eval_rigid_tau(state_in);

    }



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

void OrderSolver::allocate_state_aux_vars() {

    if (model_.num_bodies == 0)
        return;

    // Joint-space runtime buffers

    // qdd: same size as joint_qd, initialized to zero
    joint_qdd.assign(model_.num_joint_dof, 0.0f);

    // tau: same size as joint_qd
    joint_tau.assign(model_.num_joint_dof, 0.0f);

    // one spatial motion subspace vector per DOF
    joint_S_s.resize(model_.num_joint_dof);
    for (size_t i = 0; i < model_.num_joint_dof; ++i) {
        joint_S_s[i].setZero();
    }

    // Derived rigid-body data

    // body_q_com: same count as bodies
    body_q_com.assign(model_.num_bodies, TTransform::Identity());

    // body_I_s
    body_I_s.resize(model_.num_bodies);
    for (size_t i = 0; i < model_.num_bodies; ++i) {
        body_I_s[i].setZero();
    }

    // body_v_s
    body_v_s.resize(model_.num_bodies);
    for (size_t i = 0; i < model_.num_bodies; ++i) {
        body_v_s[i].setZero();
    }

    // body_a_s
    body_a_s.resize(model_.num_bodies);
    for (size_t i = 0; i < model_.num_bodies; ++i) {
        body_a_s[i].setZero();
    }

    // body_f_s
    body_f_s.resize(model_.num_bodies);
    for (size_t i = 0; i < model_.num_bodies; ++i) {
        body_f_s[i].setZero();
    }

    // body_ft_s
    body_ft_s.resize(model_.num_bodies);
    for (size_t i = 0; i < model_.num_bodies; ++i) {
        body_ft_s[i].setZero();
    }

    state_aux_allocated_ = true;
}

void OrderSolver::convert_body_force_com_to_origin(State& state_in) const {
    const size_t n = model_.num_bodies;
    const auto& body_pos = state_in.body_pos;
    const auto& body_rot = state_in.body_rot;
    auto& body_force = state_in.body_force;
    auto& body_torque = state_in.body_torque;

    for (size_t i = 0; i < n; ++i) {
        const auto& p = body_pos[i];
        const auto& q = body_rot[i];
        auto body_q = TTransform{p, q};

        const Vec3& f = body_force[i];
        const Vec3& tau_com = body_torque[i];

        if (f.isZero() && tau_com.isZero())
            continue;

        // const State& state_in
        TTransform com_world = body_q * body_X_com[i];
        Vec3 r_com = com_world.p;

        // shift torque from COM to world origin
        const Vec3 tau_world_origin = tau_com + r_com.cross(f);

        body_force[i] = -f;
        body_torque[i] = -tau_world_origin;
    }
}

void OrderSolver::eval_rigid_fk(State &state_in) {
    for (size_t art = 0; art < model_.num_articulation; ++art) {
        const int start = model_.articulation_start[art];
        const int end = model_.articulation_start[art + 1];

        for (int j = start; j < end; ++j) {
            compute_link_transform(j, state_in);
        }
    }
}

void OrderSolver::eval_rigid_id(const State &state_in) {
    for (size_t art = 0; art < model_.num_articulation; ++art) {
        const int start = model_.articulation_start[art];
        const int end =model_.articulation_start[art + 1];

        for (int j = start; j < end; ++j) {
            compute_link_velocity(j, state_in);
        }
    }

}

void OrderSolver::eval_body_contact(State &state_in, const Contacts *contacts) {

    if (contacts == nullptr)
        return;

    const int count = contacts->rigid_contact_count();

    for (int i = 0; i < count; ++i) {
        // read contact / shapes
        const int shape_a = contacts->rigid_contact_shape0[i];
        const int shape_b = contacts->rigid_contact_shape1[i];

        // currently, thickness is margin in newton
        const float thickness_a = contacts->rigid_contact_thickness0[i];
        const float thickness_b = contacts->rigid_contact_thickness1[i];

        int body_a = -1;
        int body_b = -1;

        // average material parameters
        float ke = 0.0f;   // normal stiffness
        float kd = 0.0f;   // damping
        float kf = 0.0f;   // friction stiffness
        // float ka = 0.0f;   // adhesion cutoff distance
        float mu = 0.0f;   // friction coefficient
        int mat_nonzero = 0;

        if (shape_a >= 0) {
            ++mat_nonzero;
            ke += model_.shape_material_ke[shape_a];
            kd += model_.shape_material_kd[shape_a];
            kf += model_.shape_material_kf[shape_a];
            /*ka += model_.shape_material_ka[shape_a];*/
            mu += model_.shape_material_mu[shape_a];
            body_a = model_.shape_body[shape_a];
        }

        if (shape_b >= 0) {
            ++mat_nonzero;
            ke += model_.shape_material_ke[shape_b];
            kd += model_.shape_material_kd[shape_b];
            kf += model_.shape_material_kf[shape_b];
            /*ka += model_.shape_material_ka[shape_b];*/
            mu += model_.shape_material_mu[shape_b];
            body_b = model_.shape_body[shape_b];
        }

        if (mat_nonzero > 0) {
            const float inv = 1.0f / static_cast<float>(mat_nonzero);
            ke *= inv;
            kd *= inv;
            kf *= inv;
            // ka *= inv;
            mu *= inv;
        }

        /*// per-contact overrides, temperary no
        if (!contacts.rigid_contact_stiffness.empty()) {
            const float contact_ke = contacts.rigid_contact_stiffness[tid];
            if (contact_ke > 0.0f) ke = contact_ke;

            const float contact_kd = contacts.rigid_contact_damping[tid];
            if (contact_kd > 0.0f) kd = contact_kd;

            const float contact_mu_scale = contacts.rigid_contact_friction_scale[tid];
            if (contact_mu_scale > 0.0f) mu *= contact_mu_scale;
        }*/

        // contact normal in world space
        const Vec3 n = contacts->rigid_contact_normal[i];
        Vec3 x_a = contacts->rigid_contact_point0[i];
        Vec3 x_b = contacts->rigid_contact_point1[i];

        Vec3 r_a = Vec3::Zero();
        Vec3 r_b = Vec3::Zero();

        if (body_a >= 0) {
            const TTransform w_X_ba{state_in.body_pos[body_a], state_in.body_rot[body_a]};
            const Vec3 w_x_com_a = w_X_ba.transformPoint(model_.body_local_com[body_a]);
            x_a = w_X_ba.transformPoint(x_a) - thickness_a * n;
            r_a = x_a - w_x_com_a;
        }

        if (body_b >= 0) {
            const TTransform w_X_bb{state_in.body_pos[body_b], state_in.body_rot[body_b]};
            const Vec3 w_x_com_b = w_X_bb.transformPoint(model_.body_local_com[body_b]);
            x_b = w_X_bb.transformPoint(x_b) - thickness_b * n;
            r_b = x_b - w_x_com_b;
        }

        // signed separation / penetration
        const float d = n.dot(x_a - x_b);

        // Contact point velocities
        Vec3 v_pt_a{0.0f, 0.0f, 0.0f};
        Vec3 v_pt_b{0.0f, 0.0f, 0.0f};

        if (body_a >= 0) {
            const SpatialVec& V_a = body_v_s[body_a];
            const Vec3 v_a = V_a.segment<3>(0);
            const Vec3 w_a = V_a.segment<3>(3);
            v_pt_a = v_a + w_a.cross(x_a);
        }

        if (body_b >= 0) {
            const SpatialVec& V_b = body_v_s[body_b];
            const Vec3 v_b = V_b.segment<3>(0);
            const Vec3 w_b = V_b.segment<3>(3);
            v_pt_b = v_b + w_b.cross(x_b);
        }

        const Vec3 v_rel = v_pt_a - v_pt_b;

        // decompose relative velocity
        const float v_n = n.dot(v_rel);
        const Vec3 v_t = v_rel - n * v_n;

        // Normal force (penalty + damping)
        const float f_n = ke * d;

        // damping only when approaching
        float f_d = 0.0f;
        if (d < 0.0f && v_n < 0.0f) {
            f_d = kd * v_n;
        }

        // Smooth Coulomb friction
        Vec3 f_t{0.0f, 0.0f, 0.0f};

        if (d < 0.0f) {
            const float v_s = TY::norm_huber(v_t, friction_smoothing);
            if (v_s > 0.0f) {
                const Vec3 dir = v_t / v_s;

                // normal reaction magnitude (positive clamp value)
                const float normal_mag = -(f_n + f_d);

                if (normal_mag > 0.0f) {
                    const float viscous_mag = kf * v_s;
                    const float coulomb_cap = mu * normal_mag;
                    const float friction_mag = std::min(viscous_mag, coulomb_cap);

                    // oppose tangential motion
                    f_t = -dir * friction_mag;
                }
            }
        }

        // total contact force on body A
        const Vec3 f_total = n * (f_n + f_d) + f_t;

        if (body_a >= 0) {
            state_in.body_force[body_a] += f_total;
            state_in.body_torque[body_a] += r_a.cross(f_total);
        }

        if (body_b >= 0) {
            state_in.body_force[body_b] -= f_total;
            state_in.body_torque[body_b] -= r_b.cross(f_total);
        }


    }

}

void OrderSolver::eval_rigid_tau(const State &state_in) {

    for (size_t art = 0; art < model_.num_articulation; ++art) {
        const int start = model_.articulation_start[art];
        const int end = model_.articulation_start[art + 1];

        const int count = end - start;

        for (int offset = 0; offset < count; ++offset) {
            const int j = end - offset - 1;

            const JointType type = model_.joint_type[j];
            const int parent = model_.joint_parent[j];
            const int child  = model_.joint_child[j];

            const int dof_start   = model_.joint_qd_start[j];
            const int coord_start = model_.joint_q_start[j];

            const int lin_axis_count = model_.joint_dof_dim[j].first;
            const int ang_axis_count = model_.joint_dof_dim[j].second;

            // total force on this child subtree
            const SpatialVec& f_b_s = body_f_s[child];     // bias force
            const SpatialVec& f_t_s = body_ft_s[child];    // accumulated child-subtree force

            SpatialVec f_ext = SpatialVec::Zero();
            f_ext.segment<3>(0) = state_in.body_force[child];
            f_ext.segment<3>(3) = state_in.body_torque[child];

            const SpatialVec f_s = f_b_s + f_t_s + f_ext;

            // project to joint-space and add derives/limits/etc
            jcalc_tau(type, coord_start, dof_start, lin_axis_count, ang_axis_count, state_in.joint_q, state_in.joint_qd, f_s);

            // propagate subtree force to parent
            if (parent >= 0) {
                body_ft_s[parent] += f_s;
            }
        }
    }

}

void OrderSolver::eval_rigid_jacobian() {
    std::ranges::fill(J, 0.0f);
    for (size_t art = 0; art < model_.num_articulation; ++art) {
        const int joint_start = model_.articulation_start[art];
        const int joint_end =model_.articulation_start[art + 1];

        const int joint_count = joint_end - joint_start;

        const int J_offset = articulation_J_start[art];

        const int art_dof_start = model_.joint_qd_start[joint_start];
        const int art_dof_end   = model_.joint_qd_start[joint_end];
        const int art_dof_count = art_dof_end - art_dof_start;

        for (int i = 0; i < joint_count; ++i) {
            const int row_start = i * 6;

            int j = joint_start + i;
            while (j != -1) {
                const int joint_dof_start = model_.joint_qd_start[j];
                const int joint_dof_end   = model_.joint_qd_start[j + 1];
                const int joint_dof_count = joint_dof_end - joint_dof_start;

                for (int dof = 0; dof < joint_dof_count; ++dof) {
                    const int col = (joint_dof_start - art_dof_start) + dof;
                    const SpatialVec& S = joint_S_s[joint_dof_start + dof];

                    for (int k = 0; k < 6; ++k) {
                        const int dense_index = (row_start + k) * art_dof_count + col;
                        J[J_offset + dense_index] = S[k];
                    }
                }

                j = model_.joint_ancestor[j];
            }
        }
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

}

void OrderSolver::compute_link_transform(const int j, State &state_in) {

    // topology
    const int parent = model_.joint_parent[j];
    const int child  = model_.joint_child[j];

    // joint anchor transforms
    const TTransform& p_X_pj = model_.joint_X_p[j];   // parent body -> parent joint anchor
    const TTransform& c_X_cj = model_.joint_X_c[j];   // child  body -> child  joint anchor

    // world transform of parent anchor frame
    TTransform w_X_pj = p_X_pj;
    if (parent >= 0) {
        const TTransform w_X_p{state_in.body_pos[parent], state_in.body_rot[parent]};
        w_X_pj = w_X_p * p_X_pj;
    }

    // joint metadata
    const JointType type = model_.joint_type[j];
    const int dof_start   = model_.joint_qd_start[j];   // for axis indexing
    const int coord_start = model_.joint_q_start[j];    // for q indexing

    const int lin_axis_count = model_.joint_dof_dim[j].first;
    const int ang_axis_count = model_.joint_dof_dim[j].second;

    // transform across the joint (depends on current joint_q)
    const TTransform X_j = jcalc_transform(
        type,
        dof_start,
        lin_axis_count,
        ang_axis_count,
        state_in.joint_q,
        coord_start
    );

    // world transform of child joint anchor
    const TTransform w_X_cj = w_X_pj * X_j;

    // world transform of child body frame
    const TTransform w_X_c = w_X_cj * c_X_cj.inverse();

    // world transform of child COM frame
    const TTransform& c_X_cc = body_X_com[child];   // child body -> child COM
    const TTransform w_X_cc = w_X_c * c_X_cc;

    // store body transform
    state_in.body_pos[child] = w_X_c.p;
    state_in.body_rot[child] = w_X_c.q;

    // store COM transform
    body_q_com[child] = w_X_cc;

}

TTransform OrderSolver::jcalc_transform(const JointType type, const int dof_start, int lin_axis_count, int ang_axis_count,
    const std::vector<float> &joint_q, const int q_start) const {

    // identity helpers
    const Vec3 zero_pos = Vec3::Zero();
    const Quat q_identity = Quat::Identity();

    switch (type) {
        case JointType::PRISMATIC: {
            const float q = joint_q[q_start];
            const Vec3& axis = model_.joint_axis[dof_start];
            return TTransform{axis * q, q_identity};
        }

        case JointType::REVOLUTE: {
            const float q = joint_q[q_start];
            const Vec3& axis = model_.joint_axis[dof_start];
            return TTransform{zero_pos, quat_from_axis_angle(axis, q)};
        }

        case JointType::BALL: {
            const float qx = joint_q[q_start + 0];
            const float qy = joint_q[q_start + 1];
            const float qz = joint_q[q_start + 2];
            const float qw = joint_q[q_start + 3];

            Quat rot{qx, qy, qz, qw};
            rot.normalize();   // 建议归一化，防止数值漂
            return TTransform{zero_pos, rot};
        }

        case JointType::FIXED: {
            return TTransform{zero_pos, q_identity};
        }

        case JointType::DISTANCE:
        case JointType::FREE: {
            const float px = joint_q[q_start + 0];
            const float py = joint_q[q_start + 1];
            const float pz = joint_q[q_start + 2];

            const float qx = joint_q[q_start + 3];
            const float qy = joint_q[q_start + 4];
            const float qz = joint_q[q_start + 5];
            const float qw = joint_q[q_start + 6];

            Quat rot{qx, qy, qz, qw};
            rot.normalize();
            return TTransform{Vec3{px, py, pz}, rot};
        }

        // case JointType::D6: {}

        default:
            return TTransform::Identity();
    }
}

SpatialVec OrderSolver::jcalc_motion(const JointType type, const int lin_axis_count, const int ang_axis_count,
                                     const TTransform &w_X_pj, const std::vector<float> &joint_qd, const int qd_start) {

    // const Vec3 zero_v3{0.0f, 0.0f, 0.0f};

    switch (type) {
    case JointType::PRISMATIC: {
        const Vec3& axis = model_.joint_axis[qd_start];

        SpatialVec S_local = SpatialVec::Zero();
        S_local.segment<3>(0) = axis;   // linear
        // angular = 0

        const SpatialVec S_s = transform_twist(w_X_pj, S_local);
        const SpatialVec v_j_s = S_s * joint_qd[qd_start];

        joint_S_s[qd_start] = S_s;
        return v_j_s;
    }

    case JointType::REVOLUTE: {
        const Vec3& axis = model_.joint_axis[qd_start];

        SpatialVec S_local = SpatialVec::Zero();
        // linear = 0
        S_local.segment<3>(3) = axis;   // angular

        const SpatialVec S_s = transform_twist(w_X_pj, S_local);
        const SpatialVec v_j_s = S_s * joint_qd[qd_start];

        joint_S_s[qd_start] = S_s;
        return v_j_s;
    }

    case JointType::D6: {
        SpatialVec v_j_s = SpatialVec::Zero();

        // linear axes
        for (int k = 0; k < lin_axis_count; ++k) {
            const Vec3& axis = model_.joint_axis[qd_start + k];

            SpatialVec S_local = SpatialVec::Zero();
            S_local.segment<3>(0) = axis;

            const SpatialVec S_s = transform_twist(w_X_pj, S_local);
            joint_S_s[qd_start + k] = S_s;
            v_j_s += S_s * joint_qd[qd_start + k];
        }

        // angular axes
        for (int k = 0; k < ang_axis_count; ++k) {
            const int idx = qd_start + lin_axis_count + k;
            const Vec3& axis = model_.joint_axis[idx];

            SpatialVec S_local = SpatialVec::Zero();
            S_local.template segment<3>(3) = axis;

            const SpatialVec S_s = transform_twist(w_X_pj, S_local);
            joint_S_s[idx] = S_s;
            v_j_s += S_s * joint_qd[idx];
        }

        return v_j_s;
    }

    case JointType::BALL: {
        // three rotational dofs about x/y/z of the joint anchor frame
        SpatialVec S0_local = SpatialVec::Zero();
        SpatialVec S1_local = SpatialVec::Zero();
        SpatialVec S2_local = SpatialVec::Zero();

        S0_local.segment<3>(3) = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        S1_local.segment<3>(3) = Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        S2_local.segment<3>(3) = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

        const SpatialVec S0 = transform_twist(w_X_pj, S0_local);
        const SpatialVec S1 = transform_twist(w_X_pj, S1_local);
        const SpatialVec S2 = transform_twist(w_X_pj, S2_local);

        joint_S_s[qd_start + 0] = S0;
        joint_S_s[qd_start + 1] = S1;
        joint_S_s[qd_start + 2] = S2;

        return
            S0 * joint_qd[qd_start + 0] +
            S1 * joint_qd[qd_start + 1] +
            S2 * joint_qd[qd_start + 2];
    }

    case JointType::FIXED: {
        return SpatialVec::Zero();
    }

    case JointType::FREE:
    case JointType::DISTANCE: {
        // local 6D twist = [vx, vy, vz, wx, wy, wz]
        SpatialVec twist_local = SpatialVec::Zero();
        twist_local(0) = joint_qd[qd_start + 0];
        twist_local(1) = joint_qd[qd_start + 1];
        twist_local(2) = joint_qd[qd_start + 2];
        twist_local(3) = joint_qd[qd_start + 3];
        twist_local(4) = joint_qd[qd_start + 4];
        twist_local(5) = joint_qd[qd_start + 5];

        const SpatialVec v_j_s = transform_twist(w_X_pj, twist_local);

        // store motion subspace basis columns
        for (int k = 0; k < 6; ++k) {
            SpatialVec basis = SpatialVec::Zero();
            basis(k) = 1.0f;
            joint_S_s[qd_start + k] = transform_twist(w_X_pj, basis);
        }

        return v_j_s;
    }

    default:
        // Optional: log warning here
        // std::cerr << "jcalc_motion not implemented for joint type "
        //           << static_cast<int>(type) << "\n";
        return SpatialVec::Zero();
    }

}

void OrderSolver::jcalc_tau(JointType type, int coord_start, int dof_start, int lin_axis_count, int ang_axis_count,
    const std::vector<float> &joint_q, const std::vector<float> &joint_qd, const SpatialVec &f_s) {

    switch (type) {
        case JointType::BALL: {
            // 3 angular dofs
            for (int i = 0; i < 3; ++i) {
                const int j = dof_start + i;
                const SpatialVec& S_s = joint_S_s[j];

                joint_tau[j] =
                    -S_s.dot(f_s)
                    + model_.joint_f0[j];
            }
            return;
        }

        case JointType::FREE:
        case JointType::DISTANCE: {
            // 6 dofs
            for (int i = 0; i < 6; ++i) {
                const int j = dof_start + i;
                const SpatialVec& S_s = joint_S_s[j];

                joint_tau[j] =
                    -S_s.dot(f_s)
                    + model_.joint_f0[j];
            }
            return;
        }

        case JointType::PRISMATIC:
        case JointType::REVOLUTE:
        case JointType::D6: {
            const int axis_count = lin_axis_count + ang_axis_count;

            for (int i = 0; i < axis_count; ++i) {
                const int j = dof_start + i;

                const SpatialVec& S_s = joint_S_s[j];

                const float q = joint_q[coord_start + i];
                const float qd = joint_qd[j];

                const float lower     = model_.joint_limit_lower[j];
                const float upper     = model_.joint_limit_upper[j];
                const float limit_ke  = model_.joint_limit_ke[j];
                const float limit_kd  = model_.joint_limit_kd[j];
                const float target_ke = model_.joint_target_ke[j];
                const float target_kd = model_.joint_target_kd[j];

                const float target_pos = model_.joint_target_pos[j];
                const float target_vel = model_.joint_target_vel[j];

                const float drive_f = joint_force(
                    q, qd,
                    target_pos, target_vel,
                    target_ke, target_kd,
                    lower, upper,
                    limit_ke, limit_kd
                );

                const float t =
                    -S_s.dot(f_s)
                    + drive_f
                    + model_.joint_f0[j];

                joint_tau[j] = t;
            }
            return;

        }

        case JointType::FIXED:
        default: {
            return;
        }
    }



}

Quat OrderSolver::quat_from_axis_angle(const Vec3 &axis, const float angle) {
    const float half = 0.5f * angle;
    const float s = std::sin(half);
    const float c = std::cos(half);

    return Quat{
        axis.x() * s,
        axis.y() * s,
        axis.z() * s,
        c
    };
}

void OrderSolver::compute_link_velocity(const int j, const State &state_in) {
    const JointType type = model_.joint_type[j];
    const int child = model_.joint_child[j];
    const int parent = model_.joint_parent[j];
    const int qd_start = model_.joint_qd_start[j];

    /* all velocity in world frame */

    const TTransform& p_X_pj = model_.joint_X_p[j];

    // parent anchor frame in world space
    TTransform w_X_pj = p_X_pj;
    if (parent >= 0) {
        const TTransform w_X_p{state_in.body_pos[parent], state_in.body_rot[parent]};
        w_X_pj = w_X_p * p_X_pj;
    }

    // compute motion subspace S and velocity contribution across the joint
    const int lin_axis_count = model_.joint_dof_dim[j].first;
    const int ang_axis_count = model_.joint_dof_dim[j].second;

    // compute vj = vp + Si * dot{qi} in world frame
    const SpatialVec v_j = jcalc_motion(
    type,
    lin_axis_count,
    ang_axis_count,
    w_X_pj,
    state_in.joint_qd,
    qd_start);

    // parent velocity / bias acceleration
    SpatialVec v_parent = SpatialVec::Zero();
    SpatialVec a_parent = SpatialVec::Zero();

    if (parent >= 0) {
        v_parent = body_v_s[parent];
        a_parent = body_a_s[parent];
    }

    // body spatial velocity and bias acceleration
    const SpatialVec v_s = v_parent + v_j;
    const SpatialVec a_s = a_parent + spatial_cross(v_s, v_j);
    // full forward dynamics would add + S * qdd later; here this is bias acceleration only

    // current COM world transform
    const TTransform& w_X_cc = body_q_com[child];

    // model-space / rest-space spatial inertia
    const Mat66& I_m = body_I_m[child];

    // mass from linear-first spatial inertia layout: top-left = m*I3
    const float m = I_m(0, 0);

    // gravity for the world this body belongs to
    const Vec3& g = model_.gravity_;

    const Vec3 f_g = g * m;
    const Vec3 r_com = w_X_cc.p;   // child body COM world position

    SpatialVec f_g_s = SpatialVec::Zero();
    f_g_s.segment<3>(0) = f_g;                  // force
    f_g_s.segment<3>(3) = r_com.cross(f_g);     // torque about world origin

    // transform model/local spatial inertia to current solver/world expression
    const Mat66 I_s = transform_spatial_inertia(w_X_cc, I_m);

    // bias force: I*a + v x* (I*v)
    const SpatialVec Iv = I_s * v_s;
    const SpatialVec f_b_s = I_s * a_s + spatial_cross_dual(v_s, Iv);

    // store outputs
    body_v_s[child] = v_s;
    body_a_s[child] = a_s;
    body_f_s[child] = f_b_s - f_g_s;
    body_I_s[child] = I_s;
}

SpatialVec OrderSolver::spatial_cross(const SpatialVec &a, const SpatialVec &b) {
    const Eigen::Vector3f v_a = a.segment<3>(0);
    const Eigen::Vector3f w_a = a.segment<3>(3);

    const Eigen::Vector3f v_b = b.segment<3>(0);
    const Eigen::Vector3f w_b = b.segment<3>(3);

    SpatialVec out = SpatialVec::Zero();

    // angular part
    out.segment<3>(3) = w_a.cross(w_b);

    // linear part
    out.segment<3>(0) = w_a.cross(v_b) + v_a.cross(w_b);

    return out;
}

SpatialVec OrderSolver::spatial_cross_dual(const SpatialVec &a, const SpatialVec &b) {

    const Eigen::Vector3f v_a = a.segment<3>(0);
    const Eigen::Vector3f w_a = a.segment<3>(3);

    const Eigen::Vector3f v_b = b.segment<3>(0);
    const Eigen::Vector3f w_b = b.segment<3>(3);

    SpatialVec out = SpatialVec::Zero();

    // linear part
    out.segment<3>(0) = w_a.cross(v_b);

    // angular part
    out.segment<3>(3) = w_a.cross(w_b) + v_a.cross(v_b);

    return out;
}

Mat66 OrderSolver::transform_spatial_inertia(const TTransform &w_X_cc, const Mat66 &I_m) {
    const TTransform t_inv = w_X_cc.inverse();

    const Mat3 R = t_inv.q.toRotationMatrix();
    const Mat3 S = TY::Skew(t_inv.p) * R;
    Mat66 T = Mat66::Zero();

    // Top-left block = R
    T.block<3,3>(0,0) = R;

    // Top-right block = S = skew(p) * R
    T.block<3,3>(0,3) = S;

    // Bottom-left block = 0
    // (already zero)

    // Bottom-right block = R
    T.block<3,3>(3,3) = R;

    return T.transpose() * I_m * T;
}

SpatialVec OrderSolver::transform_twist(const TTransform &X, const SpatialVec &twist_local) {
    const Vec3 v = twist_local.segment<3>(0);
    const Vec3 w = twist_local.segment<3>(3);

    const Mat3 R = X.q.toRotationMatrix();
    const Eigen::Vector3f p = X.p;

    const Eigen::Vector3f w_out = R * w;
    const Eigen::Vector3f v_out = R * v + p.cross(w_out);

    SpatialVec out = SpatialVec::Zero();
    out.segment<3>(0) = v_out;
    out.segment<3>(3) = w_out;
    return out;
}

float OrderSolver::joint_force(const float q, const float qd, const float joint_target_pos,
                               const float joint_target_vel, const float target_ke, const float target_kd,
                               const float limit_lower, const float limit_upper, const float limit_ke,
                               const float limit_kd) {
    float limit_f = 0.0f;
    float damping_f = 0.0f;
    float target_f = 0.0f;

    // PD target control (active only when within limits)
    target_f = target_ke * (joint_target_pos - q)
             + target_kd * (joint_target_vel - qd);

    // If limit violated: apply limit restoration force and disable target control
    if (q < limit_lower) {
        limit_f = limit_ke * (limit_lower - q);
        damping_f = -limit_kd * qd;
        target_f = 0.0f;
    }
    else if (q > limit_upper) {
        limit_f = limit_ke * (limit_upper - q);
        damping_f = -limit_kd * qd;
        target_f = 0.0f;
    }

    return limit_f + damping_f + target_f;
}

