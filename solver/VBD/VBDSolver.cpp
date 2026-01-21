//
// Created by tianyan on 12/22/25.
//

#include <stdexcept>
#include <iostream>
#include "VBDSolver.h"
#include "Debugger.hpp"
#include "Math.hpp"

inline void AssignOffsets(const size_t num_nodes, std::vector<uint32_t>& offsets) {
    offsets.assign(num_nodes + 1, 0u);
}

inline Mat3 Cofactor(const Mat3& F){
    const Vec3 f0 = F.col(0);
    const Vec3 f1 = F.col(1);
    const Vec3 f2 = F.col(2);

    Mat3 C;
    C.col(0) = f1.cross(f2);
    C.col(1) = f2.cross(f0);
    C.col(2) = f0.cross(f1);
    return C; // == cof(F)
}


// -------- friction (IPC-like smooth) --------
inline void compute_projected_isotropic_friction_ipc(
    const float mu, const float fn, const Vec3& n_unit,
    const Vec3& rel_translation, const float eps_u,
    Vec3& f_out, Mat3& H_out) {
    // P = I - n n^T
    const Mat3 P = Mat3::Identity() - n_unit * n_unit.transpose();

    // tangential relative displacement
    const Vec3 t = P * rel_translation;
    const float t2 = t.squaredNorm();
    const float d  = std::sqrt(t2 + eps_u * eps_u);

    if (!(std::isfinite(d)) || d <= 0.0f || fn <= 0.0f || mu <= 0.0f) {
        f_out.setZero();
        H_out.setZero();
        return;
    }

    const float k = mu * fn;         // friction magnitude scale
    const float inv_d  = 1.0f / d;
    const float inv_d3 = inv_d * inv_d * inv_d;

    // force = -∂φ/∂x,  φ = k * sqrt(||t||^2 + eps^2)
    f_out = -k * (t * inv_d);

    // Hessian (PSD for this regularizer)
    // H = k * ( P/d - (t t^T)/d^3 )
    H_out = k * (P * inv_d - (t * t.transpose()) * inv_d3);
}

void VBDSolver::Init() {

    const size_t num_nodes = model_.total_particles();
    if (inertia_.size() != num_nodes) inertia_.resize(num_nodes);
    if (prev_pos_.size() != num_nodes) prev_pos_.resize(num_nodes);

    if (model_.topology_version != topology_version_
        || adjacency_info_.vertex_faces.offsets.size() != num_nodes + 1) {
        BuildAdjacencyInfo();
        topology_version_ = model_.topology_version;

        // record surface vertex
        surface_vertices.resize(model_.num_particles, 0);
        for (const auto& tri: model_.render_tris) {
            surface_vertices[tri.vertices[0]] = 1;
            surface_vertices[tri.vertices[1]] = 1;
            surface_vertices[tri.vertices[2]] = 1;
        }
    }
}

void VBDSolver::Step(State& state_in, State& state_out, const float dt) {

    if (dt <= 0.0f) {
        return;
    }

    Init();

    forward_step(state_in, dt);
    for (int iter = 0; iter < num_iters; ++iter) {
        ScopeTimer iter_timer = dbg_ ? dbg_->timer_iteration() : ScopeTimer(nullptr);
        solve(state_in, state_out, dt);
    }
    update_velocity(state_out, dt);
}

void VBDSolver::accumulate_stvk_triangle_force_hessian(const std::span<const Vec3> pos,
    const MMaterial& mat,
    const triangle& face,
    const uint32_t vtex_order,
    Vec3& force,
    Mat3& H) {
    // advised by newton physics, evaluate_stvk_force_hessian function
    // StVK energy density: psi = mu * ||G||_F^2 + 0.5 * lambda * (trace(G))^2

    if (vtex_order > 2)
        throw std::runtime_error("vtex order is over stvk triangle limt");

    const Vec3& x0 = pos[face.vertices[0]];
    const Vec3& x1 = pos[face.vertices[1]];
    const Vec3& x2 = pos[face.vertices[2]];
    const auto mu = mat.mu();
    const auto lambda = mat.lambda();

    // Deformation gradient F = [f0, f1] (3x2 matrix as two 3D column vectors)
    const auto DmInv00 = face.Dm_inv(0,0);
    const auto DmInv01 = face.Dm_inv(0,1);
    const auto DmInv10 = face.Dm_inv(1,0);
    const auto DmInv11 = face.Dm_inv(1,1);

    // Compute F columns directly: F = [x01, x02] * tri_pose = [f0, f1]
    const Vec3 f0 = (x1 - x0) * DmInv00 + (x2 - x0) * DmInv10;
    const Vec3 f1 = (x1 - x0) * DmInv01 + (x2 - x0) * DmInv11;

    // Green strain tensor: G = 0.5(F^T F - I) = [[G00, G01], [G01, G11]] (symmetric 2x2)
    const auto f0_dot_f0 = f0.dot(f0);
    const auto f1_dot_f1 = f1.dot(f1);
    const auto f0_dot_f1 = f0.dot(f1);

    const auto G00 = 0.5f * (f0_dot_f0 - 1.0f);
    const auto G11 = 0.5f * (f1_dot_f1 - 1.0f);
    const auto G01 = 0.5f * f0_dot_f1;

    // Frobenius norm squared of Green strain: ||G||_F^2 = G00^2 + G11^2 + 2 * G01^2
    float G_frobenius_sq = G00 * G00 + G11 * G11 + 2.0f * G01 * G01;
    if (G_frobenius_sq < 1.0e-20) {
        return;
    }

    const float trace_G = G00 + G11;

    // First Piola-Kirchhoff stress tensor (StVK model)
    // PK1 = 2*mu*F*G + lambda*trace(G)*F = [PK1_col0, PK1_col1] (3x2)
    const auto lambda_trace_G = lambda * trace_G;
    const auto two_mu = 2.0f * mu;

    const Vec3 PK1_col0 = f0 * (two_mu * G00 + lambda_trace_G) + f1 * (two_mu * G01);
    const Vec3 PK1_col1 = f0 * (two_mu * G01) + f1 * (two_mu * G11 + lambda_trace_G);

    const auto mask0 = static_cast<float>(vtex_order == 0);
    const auto mask1 = static_cast<float>(vtex_order == 1);
    const auto mask2 = static_cast<float>(vtex_order == 2);

    // Deformation gradient derivatives w.r.t. current vertex position
    const auto df0_dx = DmInv00 * (mask1 - mask0) + DmInv10 * (mask2 - mask0);
    const auto df1_dx = DmInv01 * (mask1 - mask0) + DmInv11 * (mask2 - mask0);

    // Force via chain rule: force = -(dpsi/dF) : (dF/dx)
    const Vec3 dpsi_dx = PK1_col0 * df0_dx + PK1_col1 * df1_dx;
    Vec3 delta_force = -dpsi_dx;

    // Hessian computation using Cauchy-Green invariants
    const auto df0_dx_sq = df0_dx * df0_dx;
    const auto df1_dx_sq = df1_dx * df1_dx;
    const auto  df0_df1_cross = df0_dx * df1_dx;

    const auto Ic = f0_dot_f0 + f1_dot_f1;
    const auto two_dpsi_dIc = -mu + (0.5 * Ic - 1.0) * lambda;
    const Mat3 I33 = Mat3::Identity();

   const Mat3 f0_outer_f0 = f0 * f0.transpose();
   const Mat3 f1_outer_f1 = f1 * f1.transpose();
   const Mat3 f0_outer_f1 = f0 * f1.transpose();
   const Mat3 f1_outer_f0 = f1 * f0.transpose();

   const Mat3 H_IIc00_scaled = mu * (f0_dot_f0 * I33 + 2.0 * f0_outer_f0 + f1_outer_f1);
   const Mat3 H_IIc11_scaled = mu * (f1_dot_f1 * I33 + 2.0 * f1_outer_f1 + f0_outer_f0);
   const Mat3 H_IIc01_scaled = mu * (f0_dot_f1 * I33 + f1_outer_f0);

    // d2(psi)/dF^2 components
   const Mat3 d2E_dF2_00 = lambda * f0_outer_f0 + two_dpsi_dIc * I33 + H_IIc00_scaled;
   const Mat3 d2E_dF2_01 = lambda * f0_outer_f1 + H_IIc01_scaled;
   const Mat3 d2E_dF2_11 = lambda * f1_outer_f1 + two_dpsi_dIc * I33 + H_IIc11_scaled;

    // Chain rule: H = (dF/dx)^T * (d2(psi)/dF^2) * (dF/dx)
   Mat3 delta_hessian = df0_dx_sq * d2E_dF2_00 + df1_dx_sq * d2E_dF2_11 +
       df0_df1_cross * (d2E_dF2_01 + d2E_dF2_01.transpose());

    force += delta_force * face.rest_area;
    H += delta_hessian * face.rest_area;
}

void VBDSolver::accumulate_dihedral_angle_based_bending_force_hessian(const std::span<const Vec3> pos,
    const MMaterial& mat,
    const edge& e,
    const uint32_t vtex_order,
    Vec3& force,
    Mat3& H) {
    // advised by function with the same name in newton physics.
    constexpr float eps = 1e-6f;

    const auto& x0 = pos[e.vertices[0]];
    const auto& x1 = pos[e.vertices[1]];
    const auto& x2 = pos[e.vertices[2]];
    const auto& x3 = pos[e.vertices[3]];

    // Compute edge vectors
    const Vec3 x02 = x2 - x0;
    const Vec3 x03 = x3 - x0;
    const Vec3 x12 = x2 - x1;
    const Vec3 x13 = x3 - x1;
    const Vec3 edge_line = x3 - x2;

    // Compute normals
    const Vec3 n1 = x02.cross(x03);
    const Vec3 n2 = x13.cross(x12);

    const float n1_norm = n1.norm();
    const float n2_norm = n2.norm();
    const float e_norm = edge_line.norm();

    if (n1_norm < eps || n2_norm < eps) return;

    const Vec3 n1_hat = n1 / n1_norm;
    const Vec3 n2_hat = n2 / n2_norm;
    const Vec3 e_hat = edge_line / e_norm;

    const auto sin_theta = (n1_hat.cross(n2_hat)).dot(e_hat);
    const auto cos_theta = n1_hat.dot(n2_hat);

    const auto theta = std::atan2(sin_theta, cos_theta);
    const auto k = mat.bend_stiff() * e.rest_length;
    const auto dE_dtheta = k * (theta - e.rest_theta);

    // Pre-compute skew matrices (shared across all angle derivative computations)
    Mat3 skew_e = TY::Skew(edge_line);
    Mat3 skew_x02 = TY::Skew(x02);
    Mat3 skew_x03 = TY::Skew(x03);
    Mat3 skew_x12 = TY::Skew(x12);
    Mat3 skew_x13 = TY::Skew(x13);
    Mat3 skew_n1 = TY::Skew(n1_hat);
    Mat3 skew_n2 = TY::Skew(n2_hat);

    // Compute the derivatives of unit normals with respect to each vertex; required for computing angle derivatives
    const auto dn1hat_dx0 = TY::ComputeNormalizedVectorDerivative(n1_norm, n1_hat, skew_e);
    const Mat3 dn2hat_dx0 = Mat3::Zero();

    const Mat3 dn1hat_dx1 = Mat3::Zero();
    const Mat3 dn2hat_dx1 = TY::ComputeNormalizedVectorDerivative(n2_norm, n2_hat, -skew_e);

    const Mat3 dn1hat_dx2 = TY::ComputeNormalizedVectorDerivative(n1_norm, n1_hat, -skew_x03);
    const Mat3 dn2hat_dx2 = TY::ComputeNormalizedVectorDerivative(n2_norm, n2_hat, skew_x13);

    const Mat3 dn1hat_dx3 = TY::ComputeNormalizedVectorDerivative(n1_norm, n1_hat, skew_x02);
    const Mat3 dn2hat_dx3 = TY::ComputeNormalizedVectorDerivative(n2_norm, n2_hat, -skew_x12);

    // Compute all angle derivatives (required for damping)
    const Vec3 dtheta_dx0 = TY::ComputeAngleDerivative(
    n1_hat, n2_hat, e_hat, dn1hat_dx0, dn2hat_dx0, sin_theta, cos_theta, skew_n1, skew_n2
    );

    const Vec3 dtheta_dx1 = TY::ComputeAngleDerivative(
    n1_hat, n2_hat, e_hat, dn1hat_dx1, dn2hat_dx1, sin_theta, cos_theta, skew_n1, skew_n2
    );

    const Vec3 dtheta_dx2 = TY::ComputeAngleDerivative(
    n1_hat, n2_hat, e_hat, dn1hat_dx2, dn2hat_dx2, sin_theta, cos_theta, skew_n1, skew_n2
    );

    const Vec3 dtheta_dx3 = TY::ComputeAngleDerivative(
    n1_hat, n2_hat, e_hat, dn1hat_dx3, dn2hat_dx3, sin_theta, cos_theta, skew_n1, skew_n2
    );

    // Use float masks for branch-free selection
    const auto mask0 = static_cast<float>(vtex_order == 0);
    const auto mask1 = static_cast<float>(vtex_order == 1);
    const auto mask2 = static_cast<float>(vtex_order == 2);
    const auto mask3 = static_cast<float>(vtex_order == 3);

    // Select the derivative for the current vertex without branching
    const Vec3 dtheta_dx = dtheta_dx0 * mask0 + dtheta_dx1 * mask1 + dtheta_dx2 * mask2 + dtheta_dx3 * mask3;

    // Compute elastic force and hessian
    const Vec3 bending_force = -dE_dtheta * dtheta_dx;
    const Mat3 bending_hessian = k * dtheta_dx * dtheta_dx.transpose();

    force += bending_force;
    H += bending_hessian;
}


void VBDSolver::accumulate_neo_hookean_tetrahedron_force_hessian(const std::span<const Vec3> pos, const MMaterial &mat,
    const tetrahedron &tet, const uint32_t vtex_order, Vec3 &force, Mat3 &H, const size_t tet_id) const{

    const float mu     = mat.mu();
    const float lambda = mat.lambda();

    const float V0 = tet.restVolume;
    const Mat3& invDm = tet.Dm_inv;                 // Dm^{-1}
    const Mat3  invDmT = invDm.transpose();         // Dm^{-T}

    // ---- gather positions ----
    const Vec3& x0 = pos[tet.vertices[0]];
    const Vec3& x1 = pos[tet.vertices[1]];
    const Vec3& x2 = pos[tet.vertices[2]];
    const Vec3& x3 = pos[tet.vertices[3]];

    // ---- Ds ----
    Mat3 Ds;
    Ds.col(0) = x1 - x0;
    Ds.col(1) = x2 - x0;
    Ds.col(2) = x3 - x0;

    // ---- F ----
    const Mat3 F = Ds * invDm;

    // ---- cof(F) and J ----
    const Mat3 cofF = Cofactor(F);
    const float J = F.col(0).dot(cofF.col(0)); // det(F)

    if (!std::isfinite(J)) {
        return;
    }

    // ---- build wi (grad Ni) ----
    const Vec3 w1 = invDmT.col(0);
    const Vec3 w2 = invDmT.col(1);
    const Vec3 w3 = invDmT.col(2);
    const Vec3 w0 = -(w1 + w2 + w3);

    const auto m0 = static_cast<float>(vtex_order == 0u);
    const auto m1 = static_cast<float>(vtex_order == 1u);
    const auto m2 = static_cast<float>(vtex_order == 2u);
    const auto m3 = static_cast<float>(vtex_order == 3u);

    const Vec3 wi = w0*m0 + w1*m1 + w2*m2 + w3*m3;

    // ---- n_i = dJ/dx_i = cof(F) * w_i ----
    Vec3 ni = cofF * wi;
    if (!ni.allFinite()) {
        return;
    }

    // ------------------------------------------------------------
    // Log-J Neo-Hookean:
    //   P = mu (F - F^{-T}) + lambda ln(J) F^{-T}
    //   with F^{-T} = cof(F)/J
    //
    // NOTE: model is only physically defined for J > 0.
    // We clamp J to a small positive value to avoid NaNs/Infs.
    // For robust inversion handling, add a dedicated inversion-safe method later.
    // ------------------------------------------------------------
    constexpr float J_eps = 0.1f;          // numerical floor
    const float J_safe = std::max(J, J_eps);
    const float logJ   = std::log(J_safe);

    // F^{-T} via cofactor
    const Mat3 FinvT = cofF / J_safe;

    // P = mu*F + (lambda*logJ - mu) * F^{-T}
    const float c = (lambda * logJ - mu);
    const Mat3 P  = mu * F + c * FinvT;

    // ---- force contribution (negative gradient) ----
    // f_i = -V0 * (P * w_i)
    const Vec3 fi = -V0 * (P * wi);

    // ---- Hessian diagonal block (VBD-friendly SPD) ----
    const float wi2 = wi.squaredNorm();

    // constexpr float diag_eps = 1.0e-10f;
    Mat3 Hi = (V0 * (mu * wi2)) * Mat3::Identity();
    // Hi.diagonal().array() += diag_eps;

    // volumetric curvature for phi(J) = -mu ln J + (lambda/2)(ln J)^2:
    // phi''(J) = (mu + lambda(1 - ln J)) / J^2
    float phi_dd = (mu + lambda * (1.0f - logJ)) / (J_safe * J_safe);

    // To keep H_ii SPD (like Gaia's PSD filtering), clamp negative curvature.
    // If you want the exact (possibly indefinite) Hessian, remove the max().
    phi_dd = std::max(0.0f, phi_dd);

    Hi += (V0 * phi_dd) * (ni * ni.transpose());

    if (!fi.allFinite() || !Hi.allFinite()) {
        return;
    }

    force += fi;
    H += Hi;

    if (dbg_) {
        const auto signed_V = Ds.col((0)).dot(Ds.col(1).cross(Ds.col(2))) * (1.0f / 6.0f);
        dbg_->inspect_tet(tet_id, J, signed_V);
    }
}

void VBDSolver::evaluate_static_plane_particle_contact(const Vec3 &x, const Vec3 &x_prev, const Vec3 &plane_point,
                                                       const Vec3 &plane_n_unit, const float radius, const float ke,
                                                       const float kd_ratio, const float friction_mu, const float friction_epsilon,
                                                       const float dt, Vec3 &f_out, Mat3 &H_out) const{
    const Vec3& n = plane_n_unit;

    // signed distance along n: s = n·(x - p)
    const float s = n.dot(x - plane_point);

    // penetration depth: d = r - s
    float d = radius - s;

    if (!(std::isfinite(d)) || d <= 0.0f) {
        f_out.setZero();
        H_out.setZero();
        return;
    }

    dbg_->record_collision();

    // Optional: clamp penetration to avoid extreme impulses when tunneling
    d = std::min(d, 0.01f); // tune in length units (or 0.2*avg_edge_length)

    // Normal spring
    const float fn = ke * d;
    Vec3 f = n * fn;
    Mat3 K = ke * (n * n.transpose());

    // Finite-difference displacement over dt (particle vs static plane)
    const Vec3 dx = x - x_prev;

    // Normal damping only when approaching: dot(n, dx) < 0
    if (n.dot(dx) < 0.0f) {
        const float dt_safe = std::max(dt, 1.0e-8f);
        const float damping_coeff = kd_ratio * ke; // Newton-style
        const float c_over_dt = damping_coeff / dt_safe;

        const Mat3 Kd = c_over_dt * (n * n.transpose());
        K += Kd;
        f -= Kd * dx; // = -c v_n n
    }

    // Friction (projected + regularized)
    if (friction_mu > 0.0f) {
        const float eps_u = friction_epsilon * dt; // Newton uses eps*dt
        Vec3 ff; Mat3 Kf;
        compute_projected_isotropic_friction_ipc(
            friction_mu, fn, n, dx /* relative_translation */, eps_u, ff, Kf
            );
        f += ff;
        K += Kf;
    }

    f_out = f;
    H_out = K;
}


void VBDSolver::forward_step(State& state_in, const float dt) {
    const size_t num_nodes = model_.total_particles();
    const auto& gravity = model_.gravity_;

    for (size_t i = 0; i < num_nodes; ++i) {
        prev_pos_[i] = state_in.particle_pos[i];

        const float inv_mass = model_.particle_inv_mass[i];
        if (inv_mass == 0) {
            inertia_[i] = state_in.particle_pos[i];
            continue;
        }

        const Vec3 vel_new = state_in.particle_vel[i] + (state_in.particle_force[i] * inv_mass * dt) + gravity * dt;
        state_in.particle_pos[i] = state_in.particle_pos[i] + vel_new * dt;
        inertia_[i] = state_in.particle_pos[i];
    }
}

void VBDSolver::solve(State& state_in, State& state_out, const float dt) const {

    if (&state_in == &state_out) {
        throw std::runtime_error("VBDSolver::Step requires distinct state_in/state_out.");
    }

    const auto num_nodes = model_.total_particles();

    // Plane: xz ground => point (0,0,0), normal +Y
    const Vec3 plane_p(0.0f, 0.0f, 0.0f);
    const Vec3 plane_n(0.0f, 1.0f, 0.0f);

    // No per-particle radius yet: use a global effective radius
    // Suggest: ~0.25~0.5 * avg_edge_length to avoid deep penetration
    const float radius = 0.15f * 0.1;  // need eigen length

    // Contact stiffness scaling: ke ~ factor * m/dt^2 keeps behavior stable across dt
    const float ke_factor = 1.0f;

    // Newton uses damping_coeff = kd_ratio * ke
    const float kd_ratio = 0.02f;  // start tiny (0~0.05)

    // friction
    const float mu_fric = 0.5f;    // start with 0.0 then enable
    const float eps_fric = 0.01f * 0.1; // length scale, need eigen length

    for (size_t vtex_id = 0; vtex_id < num_nodes; ++vtex_id) {
        auto& pos = state_in.particle_pos[vtex_id];
        auto& pos_new = state_out.particle_pos[vtex_id];
        const auto& inv_mass = model_.particle_inv_mass[vtex_id];

        if (inv_mass <= 0.0f) {
            pos_new = pos;
            continue;
        }

        const auto& face_adjacency = adjacency_info_.vertex_faces;
        const auto& edge_adjacency = adjacency_info_.vertex_edges;
        const auto& tet_adjacency = adjacency_info_.vertex_tets;

        const Vec3 inertia_force = -(pos - inertia_[vtex_id]) / (inv_mass * dt * dt);
        const Mat3 inertia_hessian = Mat3::Identity() / (inv_mass * dt * dt);

        Vec3 contact_force = Vec3::Zero();
        Mat3 contact_hessian = Mat3::Zero();

        if (surface_vertices[vtex_id]) {

            const float m = 1.0f / inv_mass;
            const float ke = ke_factor * m / (dt * dt);

            evaluate_static_plane_particle_contact(
                pos,                       // current iterate position
                prev_pos_[vtex_id],      // finite-diff reference
                plane_p, plane_n,
                radius,
                ke, kd_ratio,
                mu_fric, eps_fric,
                dt,
                contact_force, contact_hessian
            );
        }

        // dihedral_angle
        Vec3 dihedral_angle_force = Vec3::Zero();
        Mat3 dihedral_angle_hessian = Mat3::Zero();

        // stvk
        Vec3 stvk_tri_force = Vec3::Zero();
        Mat3 stvk_tri_hessian = Mat3::Zero();

        // neo_hookean
        Vec3 NH_force = Vec3::Zero();
        Mat3 NH_hessian = Mat3::Zero();

        {
            ScopeTimer gradient_timer = dbg_ ? dbg_->timer_gradient() : ScopeTimer(nullptr);
            for (uint32_t f = face_adjacency.begin(vtex_id); f < face_adjacency.end(vtex_id); ++f) {
                const auto pack = face_adjacency.incidents[f];
                const auto face_id = AdjacencyCSR::unpack_id(pack);
                const auto order = AdjacencyCSR::unpack_order(pack);
                const auto& face = model_.tris[face_id];
                accumulate_stvk_triangle_force_hessian(state_in.particle_pos, material_, face, order, stvk_tri_force, stvk_tri_hessian);
            }

            for (uint32_t e = edge_adjacency.begin(vtex_id); e < edge_adjacency.end(vtex_id); ++e) {
                const auto pack = edge_adjacency.incidents[e];
                const auto edge_id = AdjacencyCSR::unpack_id(pack);
                const auto order = AdjacencyCSR::unpack_order(pack);
                const auto& edge = model_.edges[edge_id];
                accumulate_dihedral_angle_based_bending_force_hessian(state_in.particle_pos, material_, edge, order, dihedral_angle_force, dihedral_angle_hessian);
            }

            for (uint32_t t = tet_adjacency.begin(vtex_id); t < tet_adjacency.end(vtex_id); ++t) {
                const auto pack = tet_adjacency.incidents[t];
                const auto tet_id = AdjacencyCSR::unpack_id(pack);
                const auto order = AdjacencyCSR::unpack_order(pack);
                const auto& tet = model_.tets[tet_id];
                accumulate_neo_hookean_tetrahedron_force_hessian(state_in.particle_pos, material_, tet, order, NH_force, NH_hessian, tet_id);
            }

        }

        Vec3 dx{};

        Vec3 force = inertia_force + dihedral_angle_force + stvk_tri_force + NH_force + contact_force;
        Mat3 hessian = inertia_hessian + dihedral_angle_hessian + stvk_tri_hessian + NH_hessian + contact_hessian;

        {
            ScopeTimer liner_solve_timer = dbg_ ? dbg_->timer_linear_solve() : ScopeTimer(nullptr);
            dx = TY::SolveSPDOrRegularize(hessian, force);
        }

        // debug
        if (dbg_) {
            auto penetration = std::max(-(pos.y()-radius), 0.0f);
            dbg_->inspect_vertex(vtex_id, force, hessian, dx.norm(), penetration, .1);
            if (dbg_->stop_requested())
                dbg_->record_force_hessian(inertia_force, dihedral_angle_force,
                    stvk_tri_force, NH_force, contact_force, force,
                    inertia_hessian, dihedral_angle_hessian, stvk_tri_hessian,
                    NH_hessian, contact_hessian, hessian);
        }

        /*const float maxStep = 0.05f * model_->avg_edge_length; // model average edge length
         *float n = dx.norm();
         *if (n > maxStep) dx *= (maxStep / n);*/

        pos_new = pos + dx;


        // if parallel with color group, need to copy new pos back to state_in to satisfy GS.
        // state_in.pos = state_out.pos ...
        pos = pos_new;

    }
}

void VBDSolver::update_velocity(State& state_out, const float dt) const {

    const auto num_nodes = model_.total_particles();

    for (size_t i = 0; i < num_nodes; ++i) {
        state_out.particle_vel[i] = (state_out.particle_pos[i] - prev_pos_[i]) / dt ;
    }
}

void VBDSolver::set_self_collision() {
    if (detector_) return;
    detector_ = std::make_unique<TriMeshCollisionDetector>(model_, adjacency_info_);
}

void VBDSolver::BuildAdjacencyInfo() {
    const size_t num_nodes = model_.total_particles();
    if (!model_.edges.empty()) {
        BuildVertexIncidentCSR(
            num_nodes,
            model_.edges,
            4u,
            [](const edge& e, uint32_t k) { return static_cast<uint32_t>(e.vertices[k]); },
            adjacency_info_.vertex_edges
        );
    }
    else {
        AssignOffsets(num_nodes, adjacency_info_.vertex_edges.offsets);
        adjacency_info_.vertex_edges.incidents.clear();
    }

    if (!model_.tris.empty()) {
        BuildVertexIncidentCSR(
            num_nodes,
            model_.tris,
            3u,
            [](const triangle& t, uint32_t k) { return static_cast<uint32_t>(t.vertices[k]); },
            adjacency_info_.vertex_faces
        );
    }
    else {
        AssignOffsets(num_nodes, adjacency_info_.vertex_faces.offsets);
        adjacency_info_.vertex_faces.incidents.clear();
    }

    if (!model_.tets.empty()) {
        BuildVertexIncidentCSR(
            num_nodes,
            model_.tets,
            4u,
            [](const tetrahedron& t, uint32_t k) { return static_cast<uint32_t>(t.vertices[k]); },
            adjacency_info_.vertex_tets
        );
    }
    else {
        AssignOffsets(num_nodes, adjacency_info_.vertex_tets.offsets);
        adjacency_info_.vertex_tets.incidents.clear();
    }
}

































