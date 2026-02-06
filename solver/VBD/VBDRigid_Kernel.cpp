//
// Created by tianyan on 2/4/26.
//

#include "Math.hpp"
#include "VBDSolver.h"

inline Vec3 compute_angular_velocity(const Quat& q_now, const Quat& q_prev, double dt) {
    // 1. Normalize inputs
    Quat q1 = q_now.normalized();
    Quat q0 = q_prev.normalized();

    // 2. Enforce shortest-arc

    if (q1.dot(q0) < 0.0) {
        q0.coeffs() = -q0.coeffs();
    }

    // 3. dq = q1 * conj(q0)
    Quat dq = q1 * q0.conjugate();
    dq.normalize();

    // 4. Convert to Axis-Angle
    Eigen::AngleAxisf aa(dq);

    // 5. Return omega = axis * (angle / dt)
    return aa.axis() * (aa.angle() / dt);
}

void VBDSolver::init_rigid_bodies(State &state_in, const Contacts* contacts, const float dt) {

    if (model_.num_bodies == 0) return;

    forward_step_rigid_bodies(state_in, dt);
    build_body_body_contact_lists(contacts);
    warm_start_body_body_contact(contacts);

    // body - particle interation ...

}

void VBDSolver::forward_step_rigid_bodies(State &state_in, const float dt) {

    const size_t num_bodies = model_.num_bodies;
    const Vec3 g_world = model_.gravity_;

    // 预取指针，减少循环内的查找开销
    auto* pos_ptr     = state_in.body_pos.data();
    auto* rot_ptr     = state_in.body_rot.data();
    auto* lin_vel_ptr = state_in.body_lin_vel.data();
    auto* ang_vel_ptr = state_in.body_ang_vel.data();

    // 只读数据
    const auto* inv_mass_ptr = model_.body_inv_mass.data();

    for (size_t i = 0; i < num_bodies; ++i) {
        constexpr float ang_damping = 0.0f;
        // 1. 检查静态物体
        const float inv_m = inv_mass_ptr[i];
        if (inv_m == 0.0f) {
            // 静态物体：惯性目标即为当前位姿
            body_inertia_pos_[i] = pos_ptr[i];
            body_inertia_rot_[i] = rot_ptr[i];
            continue;
        }

        // 2. 准备数据
        // 直接引用 State 中的数据，避免不必要的拷贝
        const Vec3& p0 = pos_ptr[i];
        const Quat& r0 = rot_ptr[i];
        const Vec3& v0 = lin_vel_ptr[i];
        const Vec3& w0 = ang_vel_ptr[i];

        // 外部力与力矩
        const Vec3& F   = state_in.body_force[i];
        const Vec3& Tau = state_in.body_torque[i];

        // 模型参数
        const Vec3& com_local = model_.body_local_com[i];
        const Mat3& I         = model_.body_inertia[i];
        const Mat3& invI      = model_.body_inv_inertia[i];

        // 3. 记录上一步状态 (Previous State for Friction/Damping logic later)
        body_prev_pos_[i] = p0;
        body_prev_rot_[i] = r0;

        // 4. 执行积分 (In-place update 风格，避免结构体拷贝)
        Vec3 p_new, v_new, w_new;
        Quat r_new;

        integrate_rigid_body(
            p0, r0, v0, w0,
            F, Tau,
            com_local, inv_m, I, invI,
            g_world, ang_damping, dt,
            // Outputs
            p_new, r_new, v_new, w_new
        );

        // 5. 写回状态
        pos_ptr[i]     = p_new;
        rot_ptr[i]     = r_new;
        lin_vel_ptr[i] = v_new;
        ang_vel_ptr[i] = w_new;

        // 6. 更新惯性目标 (Inertial Target for AVBD solve)
        body_inertia_pos_[i] = p_new;
        body_inertia_rot_[i] = r_new;
    }
}

void VBDSolver::integrate_rigid_body(const Vec3 &x0, const Quat &r0, const Vec3 &v0, const Vec3 &w0,
                                           const Vec3 &f_ext, const Vec3 &t_ext, const Vec3 &com_local, const float inv_mass,
                                           const Mat3 &I_body, const Mat3 &inv_I_body, const Vec3 &gravity,
                                           const float angular_damping, const float dt, /*Outputs*/ Vec3 &x_out, Quat &r_out,
                                           Vec3 &v_out, Vec3 &w_out) {


    if (dt <= 1e-6f) {
        x_out = x0; r_out = r0; v_out = v0; w_out = w0;
        return;
    }

    // --- Linear Part (Semi-implicit Euler) ---
    // 1. 计算当前 World COM
    // 注意：Eigen 的旋转乘法是 q * v
    const Vec3 com_world_offset = r0 * com_local;
    const Vec3 x_com0 = x0 + com_world_offset;

    // 2. 更新线性速度 v1 = v0 + (F/m + g) * dt
    // 提示: 这里假设 f_ext 是施加在 COM 上的力。如果不是，需要先转换。
    // Warp 代码中 f 是 spatial wrench，其 top 部分通常指 COM 受力。
    const Vec3 linear_acc = f_ext * inv_mass + gravity;
    v_out = v0 + linear_acc * dt;

    // 3. 更新 COM 位置 x_com1 = x_com0 + v1 * dt
    const Vec3 x_com1 = x_com0 + v_out * dt;

    // --- Angular Part (Body Frame Integration) ---
    // Euler Equations: I * w_dot + w x (I * w) = tau

    // 1. 转到 Body Frame
    const Quat r0_inv = r0.conjugate();
    const Vec3 w_body0 = r0_inv * w0;
    const Vec3 t_body0 = r0_inv * t_ext; // 将世界系力矩转到局部系

    // 2. 计算角加速度贡献 (包含陀螺力矩)
    // Coriolis/Gyroscopic term: - w x (I * w)
    const Vec3 ang_mom = I_body * w_body0;
    const Vec3 gyro_term = w_body0.cross(ang_mom);

    // 显式积分: w_body1 = w_body0 + I_inv * (tau - w x I w) * dt
    Vec3 w_body1 = w_body0 + (inv_I_body * (t_body0 - gyro_term)) * dt;

    // 3. 应用角阻尼 (Angular Damping)
    if (angular_damping > 0.0f) {
        w_body1 *= (1.0f - angular_damping * dt);
    }

    // --- Rotation Update (Exponential Map) ---
    // q1 = q0 * exp(w_body * dt / 2)

    const float angle = w_body1.norm() * dt;
    Quat dq;
    if (angle < 1e-8f) {
        // Taylor expansion for small angles: q ≈ [1, 0.5*w*dt]
        // 保持归一化性质的二阶近似：
        // q = [1 - angle^2/8, 0.5*w*dt]
        // 但简单的线性近似+normalize通常足够：
        Vec3 half_vec = 0.5f * dt * w_body1;
        dq = Quat(1.0f, half_vec.x(), half_vec.y(), half_vec.z());
    } else {
        const Vec3 axis = w_body1.normalized(); // 避免除以 norm，直接用计算好的
        dq = Quat(Eigen::AngleAxisf(angle, axis));
    }

    r_out = r0 * dq;
    r_out.normalize(); // 防止漂移

    // --- Reconstruct State ---
    // w_world = R_new * w_body_new
    w_out = r_out * w_body1;
    // x_new = x_com1 - R_new * com_local
    x_out = x_com1 - (r_out * com_local);
}

void VBDSolver::build_body_body_contact_lists(const Contacts *contacts) {

    std::ranges::fill(body_body_contact_counts_.begin(), body_body_contact_counts_.end(), 0);

    if (contacts == nullptr)
        return;

    const size_t num_rigid_contacts = contacts->rigid_contact_count();

    for (size_t i = 0; i < num_rigid_contacts; ++i) {

        const auto shape0 = contacts->rigid_contact_shape0[i];
        const auto shape1 = contacts->rigid_contact_shape1[i];
        const auto body0 = model_.shape_body[shape0];
        const auto body1 = model_.shape_body[shape1];

        if (body0 >= 0) {
            const size_t& idx = body_body_contact_counts_[body0]++;
            if (idx < num_pre_alloc_contacts)
                body_body_contact_counts_indices_[body0 * num_pre_alloc_contacts + idx] = i;
        }

        if (body1 >= 0) {
            const size_t& idx = body_body_contact_counts_[body1]++;
            if (idx < num_pre_alloc_contacts)
                body_body_contact_counts_indices_[body1 * num_pre_alloc_contacts + idx] = i;
        }
    }
}

void VBDSolver::warm_start_body_body_contact(const Contacts *contacts) {

    if (contacts == nullptr)
        return;

    const size_t num_rigid_contacts = contacts->rigid_contact_count();

    body_body_contact_penalty_k_.resize(num_rigid_contacts);
    body_body_contact_material_ke_.resize(num_rigid_contacts);
    body_body_contact_material_kd_.resize(num_rigid_contacts);
    body_body_contact_material_mu_.resize(num_rigid_contacts);

    for (size_t i = 0; i < num_rigid_contacts; ++i) {
        const auto shape0 = contacts->rigid_contact_shape0[i];
        const auto shape1 = contacts->rigid_contact_shape1[i];

        // Cache averaged material properties (arithmetic mean for stiffness/damping, geometric for friction)
        const float avg_ke = 0.5 * (model_.shape_material_ke[shape0] + model_.shape_material_ke[shape1]);
        const float avg_kd = 0.5 * (model_.shape_material_kd[shape0] + model_.shape_material_kd[shape1]);
        const float avg_mu = std::sqrt(model_.shape_material_mu[shape0] * model_.shape_material_mu[shape1]);

        body_body_contact_material_ke_[i] = avg_ke;
        body_body_contact_material_kd_[i] = avg_kd;
        body_body_contact_material_mu_[i] = avg_mu;

        // Reset contact penalty to k_start every frame because contact indices are not persistent across frames.
        const float k_new = std::min(k_start_body_contact, avg_ke);
        body_body_contact_penalty_k_[i] = k_new;
    }
}

void VBDSolver::solve_rigid_body(State &state_in, State &state_out, const Contacts* contacts, const float dt) {

    auto& body_pos_in = state_in.body_pos;
    auto& body_rot_in = state_in.body_rot;
    auto& body_pos_out = state_out.body_pos;
    auto& body_rot_out = state_out.body_rot;

    for (size_t b = 0; b < model_.num_bodies; b++) {

        if (model_.body_inv_mass[b] <= 0.0f)
            continue;

        accumulate_rigid_body_force_hessian(b, state_in, contacts, dt);

        // Inertial force and Hessian
        const float dt_sqr_reciprocal = 1.0 / (dt * dt);

        // read body properties
        const Vec3& p_inertia = body_inertia_pos_[b];
        const Quat& q_inertia = body_inertia_rot_[b];
        const Vec3& body_com_local = model_.body_local_com[b];
        const float inv_mass = model_.body_inv_mass[b];
        const Mat3& I_body = model_.body_inertia[b];

        const Vec3& p_current = body_pos_in[b];
        const Quat& q_current = body_rot_in[b];

        // compute com positions
        Vec3 com_current = p_current + q_current * body_com_local;
        Vec3 com_inertia = p_inertia + q_inertia * body_com_local;

        // linear inertial force and hessian
        float inertial_coeff = 1 / inv_mass * dt_sqr_reciprocal;
        Vec3 f_lin = (com_inertia - com_current) * inertial_coeff;

        // compute relative rotation via quaternion difference
        // dq = q_current^-1 * q_star
        Quat q_delta = q_current.inverse() * q_inertia;

        // Enforce shortest path (w > 0) to avoid double-cover ambiguity
        if (q_delta.z() < 0.0f)
            q_delta.coeffs() = -q_delta.coeffs();

        // rotation vector
        Vec3 q_v = q_delta.vec();
        const float v_norm = q_v.norm();
        const float w = q_delta.w();

        Vec3 theta_body;
        if (v_norm < 1e-6) {
            theta_body = 2.0 * q_v;
        } else {
            // angle = 2 * atan2(|v|, w)
            const float angle = 2.0f * std::atan2(v_norm, w);
            // theta_body = axis * angle = (v / |v|) * angle
            theta_body = q_v * (angle / v_norm);
        }

        // angular inertial torque
        Vec3 tau_body = I_body * (theta_body * dt_sqr_reciprocal);
        Vec3 tau_world = q_current * tau_body;

        // Angular Hessian in world frame: use full inertia (supports off-diagonal products of inertia)
        Mat3 R_cur = q_current.toRotationMatrix();
        Mat3 I_world = R_cur * I_body * R_cur.transpose();
        Mat3 angular_hessian = I_world * dt_sqr_reciprocal;

        // Accumulate external forces (rigid contacts)
        // Read external contributions
        const Vec3& ext_force = body_force_[b];
        const Vec3& ext_torque = body_torque_[b];
        const Mat3& ext_h_aa = body_hessian_aa_[b];
        const Mat3& ext_h_al = body_hessian_al_[b];
        const Mat3& ext_h_ll = body_hessian_ll_[b];

        Vec3 f_torque = tau_world + ext_torque;
        Vec3 f_force = f_lin + ext_force;

        Mat3 h_aa = angular_hessian + ext_h_aa;
        Mat3 h_al = ext_h_al;
        Mat3 h_ll = ext_h_ll; h_ll.diagonal().array() += inertial_coeff;

        // Accumulate joint forces (constraints)
        // currently nothing
        // ...

        // Solve 6x6 block system via Schur complement
        // Regularize angular Hessian (in-place)
        const double trA = h_aa.trace() / 3.0;
        const double epsA = 1.0e-9 * (trA + 1.0);
        h_aa.diagonal().array() += static_cast<float>(epsA);

        // Factorize linear Hessian
        Eigen::LLT<Eigen::Matrix3f> llt(h_ll);
        if (llt.info() != Eigen::Success) {
            continue;
        }

        Mat3 MinvCt = llt.solve(h_al.transpose());
        Vec3 MinvF = llt.solve(f_force);

        /*Vec3 x0 = MinvCt.col(0);
        Vec3 x1 = MinvCt.col(1);
        Vec3 x2 = MinvCt.col(2);*/

        // compute  and factorize Schur complement
        Mat3 S = h_aa - (h_al * MinvCt);
        Eigen::LLT<Eigen::Matrix3f> llt_S(S);

        // Solve for angular increment
        Vec3 rhs_w = f_torque - (h_al * MinvF);
        Vec3 w_world = llt_S.solve(rhs_w);
        Vec3 x_inc = MinvF - MinvCt * w_world;

        // Update pose from increments
        // Convert angular increment to quaternion
        const float ang_mag = w_world.norm();
        Quat dq_world;
        if (ang_mag > 1e-6) {
            dq_world = Eigen::AngleAxisf(ang_mag, w_world / ang_mag);
        } else {
            Vec3 half_w = w_world * 0.5;
            dq_world = Quat(1.0, half_w.x(), half_w.y(), half_w.z());
            dq_world.normalize();
        }

        Quat q_new = dq_world * q_current;
        q_new.normalize();

        // Update position
        Vec3 com_new = com_current + x_inc;
        Vec3 pos_new = com_new - q_current * body_com_local;

        // copy back
        body_pos_out[b] = pos_new;
        body_rot_out[b] = q_new;

        // for Parallel Gauss-Seidel
        body_pos_in[b] = pos_new;
        body_rot_in[b] = q_new;

    }

    update_duals_body_body_contacts(contacts, state_out, model_.shape_body);
}

void VBDSolver::accumulate_rigid_body_force_hessian(const size_t body_idx, const State &state_in, const Contacts *contacts, float dt) {
    const size_t num_contacts = body_body_contact_counts_[body_idx];
    for (size_t c = 0; c < num_contacts; ++c) {
        const size_t contact_idx = body_body_contact_counts_indices_[body_idx * num_pre_alloc_contacts + c];

        // safety check
        if (static_cast<int>(contact_idx) >= contacts->rigid_contact_count())
            return;

        // get shape and body
        const int shape0 = contacts->rigid_contact_shape0[contact_idx];
        const int shape1 = contacts->rigid_contact_shape1[contact_idx];
        const int b0 = model_.shape_body[shape0];
        const int b1 = model_.shape_body[shape1];

        const auto body0_pos = b0 >= 0 ? state_in.body_pos[b0] : Vec3::Zero();
        const auto body1_pos = b1 >= 0 ? state_in.body_pos[b1] : Vec3::Zero();
        const auto body0_rot = b0 >= 0 ? state_in.body_rot[b0] : Quat::Identity();
        const auto body1_rot = b1 >= 0 ? state_in.body_rot[b1] : Quat::Identity();

        if (b0 != static_cast<int>(body_idx) && b1 != static_cast<int>(body_idx))
            throw std::runtime_error("VBDSolver::solve_rigid_bodies()::body shape incompact");

        // get data
        const Vec3& cp0_local = contacts->rigid_contact_point0[contact_idx];
        const Vec3& cp1_local = contacts->rigid_contact_point1[contact_idx];
        const Vec3& contact_normal = contacts->rigid_contact_normal[contact_idx];

        // transform to world space
        const Vec3 cp0_world = b0 >= 0 ? body0_rot * cp0_local + body0_pos : cp0_local;
        const Vec3 cp1_world = b1 >= 0 ? body1_rot * cp1_local + body1_pos : cp1_local;

        // compute penetration
        const float thickness = contacts->rigid_contact_thickness0[contact_idx] + contacts->rigid_contact_thickness1[contact_idx];
        const float dist = contact_normal.dot(cp1_world - cp0_world);
        const float penetration = thickness - dist;

        if (penetration < 1e-5)
            return;

        // get material parameters
        const float contact_ke = body_body_contact_penalty_k_[contact_idx];
        const float contact_kd = body_body_contact_material_kd_[contact_idx];
        const float contact_mu = body_body_contact_material_mu_[contact_idx];

        // evaluate force and hessian
        const auto &res =
            evaluate_rigid_contact_from_collision(b0, b1, body0_pos, body1_pos, body0_rot, body1_rot, cp0_local, cp1_local,
                                                  contact_normal, penetration, contact_ke, contact_kd, contact_mu, 1e-4,
                                                  dt);

        if (static_cast<int>(body_idx) == b0) {
            body_force_[body_idx] += res.force_0;
            body_torque_[body_idx] += res.torque_0;
            body_hessian_ll_[body_idx] += res.h_ll_0;
            body_hessian_al_[body_idx] += res.h_al_0;
            body_hessian_aa_[body_idx] += res.h_aa_0;
        } else {
            body_force_[body_idx] += res.force_1;
            body_torque_[body_idx] += res.torque_1;
            body_hessian_ll_[body_idx] += res.h_ll_1;
            body_hessian_al_[body_idx] += res.h_al_1;
            body_hessian_aa_[body_idx] += res.h_aa_1;
        }
    }
}

VBDSolver::RigidContactEvalResult VBDSolver::evaluate_rigid_contact_from_collision(
    int body0, int body1, const Vec3 &body0_pos, const Vec3 &body1_pos, const Quat &body0_q, const Quat &body1_q,
    const Vec3 &contact_point_a_local, const Vec3 &contact_point_b_local, const Vec3 &contact_normal,
    float penetration_depth, float contact_ke, float contact_kd, float friction_mu, float friction_epsilon, float dt) {

    Vec3 zero_vec = Vec3::Zero();
    Mat3 zero_mat = Mat3::Zero();

    // safe exit: no penetration or no stiffness,
    if (penetration_depth <= 0.0f || contact_ke <= 0.0f) {
        return {zero_vec, zero_vec, zero_mat, zero_mat, zero_mat,
                zero_vec, zero_vec, zero_mat, zero_mat, zero_mat};
    }

    // prepare body A state
    TTransform X_wa, X_wa_prev;
    Vec3 body_a_com_local;
    if (body0 < 0) {
        X_wa = TTransform::Identity();
        X_wa_prev = TTransform::Identity();
        body_a_com_local = Vec3::Zero();
    } else {
        X_wa.p = body0_pos;
        X_wa.q = body0_q;
        X_wa_prev.p = body_prev_pos_[body0];
        X_wa.q = body_prev_rot_[body0];
        body_a_com_local = model_.body_local_com[body0];
    }

    // prepare body B state
    TTransform X_wb, X_wb_prev;
    Vec3 body_b_com_local;
    if (body1 < 0) {
        X_wb = TTransform::Identity();
        X_wb_prev = TTransform::Identity();
        body_b_com_local = Vec3::Zero();
    } else {
        X_wb.p = body1_pos;
        X_wb.q = body1_q;
        X_wb_prev.p = body_prev_pos_[body1];
        X_wb.q = body_prev_rot_[body1];
        body_b_com_local = model_.body_local_com[body1];
    }

    // center of mas in world frame
    Vec3 body_a_com_world = X_wa.transformPoint(body_a_com_local);
    Vec3 body_b_com_world = X_wb.transformPoint(body_b_com_local);

    // compute contact point in world frame (current & previous)
    Vec3 cp_a_world = X_wa.transformPoint(contact_point_a_local);
    Vec3 cp_b_world = X_wb.transformPoint(contact_point_b_local);
    Vec3 cp_a_world_prev = X_wa_prev.transformPoint(contact_point_a_local);
    Vec3 cp_b_world_prev = X_wb_prev.transformPoint(contact_point_b_local);

    // compute relative movement (finite difference)
    Vec3 dx_a = cp_a_world - cp_a_world_prev;
    Vec3 dx_b = cp_b_world - cp_b_world_prev;
    Vec3 dx_rel = dx_b - dx_a;

    // compute normal force
    // Hessian K = ke * (n outer n)
    Mat3 n_outer = contact_normal * contact_normal.transpose();
    Vec3 f_total = contact_normal * (contact_ke * penetration_depth);
    Mat3 K_total = contact_ke * n_outer;

    // compute relative velocity and damping friction
    Vec3 v_rel = dx_rel / dt;
    float v_dot_n = contact_normal.dot(v_rel);

    // damping only when compressing (v_n < 0, bodies approaching)
    if (v_dot_n < 0.0f && contact_kd > 0.0f) {
        float damping_coeff = contact_kd * contact_ke;
        Vec3 damping_force = -damping_coeff * v_dot_n * contact_normal;
        Mat3 damping_hessian = (damping_coeff / dt) * n_outer;
        f_total += damping_force;
        K_total += damping_hessian;
    }

    // normal load for friction
    const float normal_load = contact_ke * penetration_depth;

    // Friction forces (isotropic, no explicit tangent basis)
    if (friction_mu > 0.0f && normal_load > 0.0f) {
        // Tangential slip (world space)
        Vec3 v_n = contact_normal * v_dot_n;
        Vec3 v_t = v_rel - v_n;
        Vec3 u = v_t * dt;
        float eps_u = friction_epsilon * dt;

        // Projected isotropic friction (no explicit tangent basis)
        Vec3 ff; Mat3 fh;
        compute_projected_isotropic_friction(friction_mu, normal_load, contact_normal, u, 1e-5, ff, fh);

        f_total += ff;
        K_total += fh;
    }

    // Split total contact force to both bodies (Newton's 3rd law)
    Vec3 force_a = -f_total;    // Force on A (opposite to normal, pushes A away from B)
    Vec3 force_b = f_total;  // Force on B (along normal, pushes B away from A)

    // Torque arms and resulting torques
    Vec3 r_a = cp_a_world - body_a_com_world;
    Vec3 r_b = cp_b_world - body_b_com_world;

    // Angular/linear coupling using contact-point Jacobian J = [-[r]x, I]
    Mat3 r_a_skew = TY::Skew(r_a);
    Mat3 r_a_skew_T_K = r_a_skew.transpose() * K_total;
    Vec3 torque_a = r_a.cross(force_a);
    Mat3 h_aa_a = r_a_skew_T_K * r_a_skew;
    Mat3 h_al_a = -r_a_skew_T_K;

    Mat3 h_ll_a = K_total;

    Mat3 r_b_skew = TY::Skew(r_b);
    Mat3 r_b_skew_T_K = r_b_skew.transpose() * K_total;
    Vec3 torque_b = r_b.cross(force_b);
    Mat3 h_aa_b = r_b_skew_T_K * r_b_skew;
    Mat3 h_al_b = -r_b_skew_T_K;

    Mat3 h_ll_b = K_total;

    return {force_a, torque_a, h_ll_a, h_al_a, h_aa_a,
                  force_b, torque_b, h_ll_b, h_al_b, h_aa_b};
}

void VBDSolver::compute_projected_isotropic_friction(const float friction_mu, const float normal_load, const Vec3 &n_unit,
                                                     const Vec3 &slip_u, const float eps_u, Vec3 &force_out, Mat3 &H_out) {

    const float dot_nu = n_unit.dot(slip_u);
    const Vec3 u_t = slip_u - n_unit * dot_nu;
    const float u_t_norm = u_t.norm();

    if (u_t_norm > 0.0f) {
        // IPC-style regularization
        float f1_SF_over_x;
        if (u_t_norm > eps_u)
            f1_SF_over_x = 1.0f / u_t_norm;
        else
            f1_SF_over_x = (-u_t_norm / eps_u + 2.0) / eps_u;

        // Factor common scalar; force aligned with u_t, Hessian proportional to projector
        float scale = friction_mu * normal_load * f1_SF_over_x;
        force_out = -(scale * u_t);
        H_out = scale * (Mat3::Identity() - n_unit * n_unit.transpose());
    } else {
        force_out = Vec3::Zero();
        H_out = Mat3::Zero();
    }
}

void VBDSolver::update_duals_body_body_contacts(const Contacts *contacts, const State &state_out, const std::vector<int> &shape_body) {

    for (int i = 0; i < contacts->rigid_contact_count(); i++) {
        // read contact geometry
        const int shape0 = contacts->rigid_contact_shape0[i];
        const int shape1 = contacts->rigid_contact_shape1[i];
        const int body0 = shape_body[shape0];
        const int body1 = shape_body[shape1];

        if (body0 < 0 && body1 < 0)
            continue;

        // read cached material stiffness
        float stiffness = body_body_contact_material_ke_[i];

        // transform contact points to world frame
        Vec3 p0_world;
        if (body0 >= 0)
            p0_world = state_out.body_pos[body0] + state_out.body_rot[body0] * contacts->rigid_contact_point0[i];
        else
            p0_world = contacts->rigid_contact_point0[i];

        Vec3 p1_world;
        if (body1 >= 0)
            p1_world = state_out.body_pos[body1] + state_out.body_rot[body1] * contacts->rigid_contact_point1[i];
        else
            p1_world = contacts->rigid_contact_point1[i];

        // Compute penetration depth (constraint violation)
        // Distance along the stored normal (normal points shape0 -> shape1)
        // dist = dot(n, p0 - p1); positive implies separation along normal
        Vec3 d = p1_world - p0_world;
        const float dist = contacts->rigid_contact_normal[i].dot(d);
        const float thickness = contacts->rigid_contact_thickness0[i] + contacts->rigid_contact_thickness1[i];
        const float penetration = std::max(0.0f, thickness - dist);

        // update penalty: k_new = min(k + beta * |C|, stiffness)
        const float k = body_body_contact_penalty_k_[i];
        const float k_new = std::min(k + 1e5f * penetration, stiffness);
        body_body_contact_penalty_k_[i] = k_new;
    }
}

void VBDSolver::update_rigid_body_vel(State &state_out, const std::vector<Vec3> &body_com_local, const float dt) const {
    const auto num_bodies = model_.num_bodies;
    for (size_t i = 0; i < num_bodies; i++) {

        const Vec3& pos = state_out.body_pos[i];
        const Vec3& pos_prev = body_prev_pos_[i];
        const Quat& q = state_out.body_rot[i];
        const Quat& q_prev = body_prev_rot_[i];

        // Compute COM positions
        const Vec3& com_local = body_com_local[i];
        const Vec3 x_com = pos + q * com_local;
        const Vec3 x_com_prev = pos_prev + q_prev * com_local;

        // linear velocity
        const Vec3 v = (x_com - x_com_prev) / dt;

        // angular velocity
        const Vec3 omega = compute_angular_velocity(q, q_prev, dt);

        state_out.body_lin_vel[i] = v;
        state_out.body_ang_vel[i] = omega;
    }
}
