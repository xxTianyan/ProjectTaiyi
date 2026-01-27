//
// Created by 徐天焱 on 2025/11/4.
//

#ifndef TAIYI_MODEL_H
#define TAIYI_MODEL_H

#include <vector>
#include <string>
#include <array>
#include <algorithm>
#include "Types.h"
#include "Collide.h"
#include "Contacts.h"

struct tetrahedron {
    std::array<VertexID, 4> vertices{0,0,0,0};
    float restVolume{};
    float restSign{};       // +1 or -1
    Mat3 Dm_inv{};

    tetrahedron(const VertexID vtex0, const VertexID vtex1, const VertexID vtex2, const VertexID vtex3,
        const Vec3& vtex0_pos, const Vec3& vtex1_pos, const Vec3& vtex2_pos, const Vec3& vtex3_pos) :
    vertices{vtex0, vtex1, vtex2, vtex3} {
        // construct Dm
        const Vec3 e1 = vtex1_pos - vtex0_pos;
        const Vec3 e2 = vtex2_pos - vtex0_pos;
        const Vec3 e3 = vtex3_pos - vtex0_pos;

        Mat3 Dm;
        Dm.col(0) = e1;
        Dm.col(1) = e2;
        Dm.col(2) = e3;

        // more stable det：det = dot(e1, cross(e2, e3))
        const auto detDm = e1.dot(e2.cross(e3));
        restSign = detDm > 0 ? 1.0f : -1.0f;

        const float absDet = std::fabs(detDm);

        // check if degenerate
        if (constexpr float kEps = 1.0e-12f; absDet < kEps)
            throw std::runtime_error("tetrahedron::compute_rest: degenerate tetrahedron (|det(Dm)| too small).");

        restVolume = absDet * (1.0f / 6.0f);

        // pre-compute
        Dm_inv  = Dm.inverse();
    };
};

struct triangle {
    std::array<VertexID, 3> vertices{0,0,0};
    float rest_area;
    Mat2 Dm_inv;

    triangle(const VertexID vtex0, const VertexID vtex1, const VertexID vtex2, const Vec3& vtex0_pos, const Vec3& vtex1_pos, const Vec3& vtex2_pos) :
    vertices{vtex0, vtex1, vtex2} {

        /*
         * advised by newton add_triangle function in builder.py
         */

        // build local 2d material basis from rest position
        const Vec3 e01 = vtex1_pos - vtex0_pos;
        const float L = e01.norm();
        constexpr float eps = 1e-8f;

        if (L < eps) {
            // Degenerate: vtex0_pos == vtex1_pos
            rest_area = 0.0f;
            Dm_inv.setZero();
            return;
        }

        const Vec3 e1 = e01 / L;

        const Vec3 e02 = vtex2_pos - vtex0_pos;
        Vec3 n = e01.cross(e02);
        const float n_len = n.norm();
        if (n_len < eps) {
            // Degenerate: collinear triangle in rest pose
            rest_area = 0.0f;
            Dm_inv.setZero();
            return;
        }

        n /= n_len;
        const Vec3 e2 = n.cross(e1);  // orthonormal within triangle plane

        // --- Rest 2D coordinates ---
        // u0 = (0,0)
        // u1 = (L,0)
        // u2 = (u,v)
        const float u = e02.dot(e1);
        const float v = e02.dot(e2);

        // Dm = [[L, u],
        //       [0, v]]
        // det(Dm) = L*v
        const float det = L * v;
        if (det < eps) {
            rest_area = 0.0f;
            Dm_inv.setZero();
            return;
        }

        // --- rest area ---
        rest_area = 0.5f * det;


        // --- explicit inverse of upper-triangular Dm ---
        // Dm^{-1} = [[ 1/L,   -u/(L*v)],
        //           [ 0,      1/v      ]]
        Dm_inv(0, 0) = 1.0f / L;
        Dm_inv(0, 1) = -u / det;  // -u/(L*v)
        Dm_inv(1, 0) = 0.0f;
        Dm_inv(1, 1) = 1.0f / v;
    };
};

struct render_trangle {
    std::array<VertexID, 3> vertices{0,0,0};
    render_trangle(const VertexID vtex0, const VertexID vtex1, const VertexID vtex2) :
    vertices{vtex0, vtex1, vtex2} {}
};

struct edge {
    // convention [opp0, opp1, edge_start, edge_end]
    std::array<VertexID, 4> vertices{0,0,0,0};
    float rest_theta;
    float rest_length;
    edge(const VertexID vtex_opp0, const VertexID vtex_opp1, const VertexID vtex_e0, const VertexID vtex_e1,
        const Vec3& vtex_opp0_pos, const Vec3& vtex_opp1_pos, const Vec3& vtex_e0_pos, const Vec3& vtex_e1_pos) :
    vertices{vtex_opp0, vtex_opp1, vtex_e0, vtex_e1} {
        rest_length = (vtex_e1_pos - vtex_e0_pos).norm();
        rest_theta = ComputeRestDihedralAngle(vtex_opp0_pos, vtex_opp1_pos, vtex_e0_pos, vtex_e1_pos);
    };

    // the same definition with newton warp atan2(sin, cos)
    static float ComputeRestDihedralAngle(const Vec3& x0, const Vec3& x1, const Vec3& x2, const Vec3& x3) {

        constexpr float eps = 1e-6f;
        const Vec3 e = x3 - x2;
        const float e_norm = e.norm();
        if (e_norm < eps) return 0.0f;

        const Vec3 x02 = x2 - x0;
        const Vec3 x03 = x3 - x0;
        const Vec3 x12 = x2 - x1;
        const Vec3 x13 = x3 - x1;

        const Vec3 n1 = x02.cross(x03);
        const Vec3 n2 = x13.cross(x12);

        const float n1_norm = n1.norm();
        const float n2_norm = n2.norm();

        if (n1_norm < eps || n2_norm < eps) return 0.0f;

        const Vec3 n1_hat = n1 / n1_norm;
        const Vec3 n2_hat = n2 / n2_norm;
        const Vec3 e_hat = e / e_norm;

        const float sin_theta = (n1_hat.cross(n2_hat)).dot(e_hat);
        const float cos_theta = n1_hat.dot(n2_hat);

        return std::atan2(sin_theta, cos_theta);
    };
};

enum class ShapeType {
    Sphere = 0,
    Box    = 1,
    Capsule= 2,
    Plane  = 3,
    // SDF etc...
};

struct range {
    size_t begin;
    size_t count;
    [[nodiscard]] size_t end() const { return begin + count;};
};

struct DeformableBodyInfo {
    std::string name;
    range particle;
    range edge;
    range tri;
    range render_tri;
    range tet;
};

struct RigidBodyInfo {
    std::string name;
    int body_id;
    range vertex;  // vertices of rigid body surface mesh
    range shapes;
    range render_tri;
};

struct State {

    // deformable body
    std::vector<Vec3> particle_pos;          // current frame pos
    std::vector<Vec3> particle_vel;          // velocity
    std::vector<Vec3> particle_force;        // force

    // rigid body
    std::vector<Vec3> body_pos;             // world position
    std::vector<Quat> body_rot;             // world orientation
    std::vector<Vec3> body_lin_vel;         // world linear velocity
    std::vector<Vec3> body_ang_vel;         // world angular velocity

    std::vector<Vec3> body_force;           // world accumulated force
    std::vector<Vec3> body_torque;          // world accumulated torque

    void resize_particle(const size_t n_nodes) {
        particle_pos.resize(n_nodes);
        particle_vel.resize(n_nodes);
        particle_force.resize(n_nodes);
    }

    void resize_bodies(const size_t n) {
        body_pos.resize(n);
        body_rot.resize(n);
        body_lin_vel.resize(n);
        body_ang_vel.resize(n);
        body_force.resize(n);
        body_torque.resize(n);
    }

};

struct MModel {
    // --- deformable body ---
    std::vector<DeformableBodyInfo> mesh_infos;
    // topology
    std::vector<tetrahedron> tets;
    std::vector<triangle> tris;
    std::vector<edge> edges;
    // particle initial date
    std::vector<Vec3> particle_pos0; // initial positions
    std::vector<Vec3> particle_vel0; // initial velocities (optional; default zero)
    std::vector<float> particle_inv_mass;

    size_t num_particles = 0;      // total number of particles

    uint64_t topology_version = 0;

    // --- rigid bodies ---
    std::vector<RigidBodyInfo> body_infos;
    std::vector<Vec3> body_pos0;
    std::vector<Quat> body_rot0;
    std::vector<Vec3> body_lin_vel0;   // optional
    std::vector<Vec3> body_ang_vel0;   // optional

    std::vector<Vec3>  body_local_com;        // COM in local/body frame
    std::vector<float> body_inv_mass;  // inv_mass==0 => static/kinematic
    std::vector<Mat3>  body_inertia; // inverse inertia in BODY frame
    std::vector<Mat3>  body_inv_inertia; // inverse inertia in BODY frame

    std::vector<Vec3> body_render_vertices;  // rigid body render surface vertex local position.

    size_t num_bodies = 0;

    // --- shape for rigid body ---
    std::vector<Vec3>  shape_pos0;          // local to body
    std::vector<Quat>  shape_rot0;          // local to body
    std::vector<int>   shape_body;          // owning body index
    std::vector<int32_t>  shape_type;       // ShapeType
    std::vector<Vec3>  shape_scale;         // non-uniform scale

    std::vector<uint32_t> body_shapes_offsets; // [num_body + 1]
    std::vector<uint32_t> body_shapes_indices; // flattened list of shape ids

    // ---- for rendering ----
    std::vector<render_trangle> render_tris;


    [[nodiscard]] State MakeState() const {
        State s;
        // ----------- particle --------------------
        s.resize_particle(num_particles);
        std::ranges::copy(particle_pos0, s.particle_pos.begin());
        // currently, no initial velocity
        std::ranges::fill(s.particle_vel, Vec3::Zero());
        std::ranges::fill(s.particle_force, Vec3::Zero());

        // -------------- rigid body ---------------------
        s.resize_bodies(num_bodies);
        std::ranges::copy(body_pos0, s.body_pos.begin());
        std::ranges::copy(body_rot0, s.body_rot.begin());
        if (body_lin_vel0.size() == num_bodies) std::ranges::copy(body_lin_vel0, s.body_lin_vel.begin());
        else std::ranges::fill(s.body_lin_vel, Vec3::Zero());

        if (body_ang_vel0.size() == num_bodies) std::ranges::copy(body_ang_vel0, s.body_ang_vel.begin());
        else std::ranges::fill(s.body_ang_vel, Vec3::Zero());

        std::ranges::fill(s.body_force,  Vec3::Zero());
        std::ranges::fill(s.body_torque, Vec3::Zero());

        return s;
    }

    // global
    Vec3 gravity_{};


    // collide
    std::unique_ptr<CollisionPipeline> collision_pipeline_;
    Contacts& Collide(const State& state_in, const CollideParams& params);

};



// void ParseMSH(const std::string& path, mesh_on_cpu* cpu_mesh);

// IndexBuffer BuildSurfaceTriangles(const std::vector<tetrahedron>& tets);

// IndexBuffer BuildSurfaceTriangles(const std::vector<triangle>& tris);



#endif //TAIYI_MODEL_H
