//
// Created by 徐天焱 on 2025/11/4.
//

#ifndef TAIYI_MODEL_H
#define TAIYI_MODEL_H

#include <vector>
#include <string>
#include <algorithm>
#include "Types.h"
#include "Collide.h"
#include "Geometry.h"
#include "Contacts.hpp"
#include "Joints.h"

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

    std::vector<Vec3> body_force;           // world external force
    std::vector<Vec3> body_torque;          // world external torque

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
    std::vector<Mat3>  body_inertia; //  inertia in BODY frame
    std::vector<Mat3>  body_inv_inertia; // inverse inertia in BODY frame

    std::vector<uint8_t> body_active_flags;   // kinematic/static-lock

    std::vector<Vec3> body_render_vertices;  // rigid body render surface vertex local position.

    size_t num_bodies = 0;

    // --- shape for rigid body ---
    std::vector<Vec3>  shape_pos0;          // local to body
    std::vector<Quat>  shape_rot0;          // local to body
    std::vector<int>   shape_body;          // owning body index
    std::vector<int>   shape_type;           // ShapeType
    std::vector<Vec3>  shape_scale;         // non-uniform scale
    std::vector<float> shape_thickness;     // shape padding
    std::vector<float> shape_collision_radius; // bounding sphere radius
    std::vector<std::pair<size_t, size_t>> shape_contact_pairs;

    std::vector<float> shape_material_ke;   // shape contact elastic stiffness, [shape_count]
    std::vector<float> shape_material_kd;  // shape contact damping stiffness
    std::vector<float> shape_material_mu;  // shape coefficient of friction
    std::vector<float> shape_contact_margin;   // per-shape margin

    std::vector<int> body_shapes_offsets{0}; // [num_body + 1]
    std::vector<int> body_shapes_indices; // flattened list of shape ids

    size_t num_shapes = 0;

    // ----------- joints ---------
    size_t num_joints = 0;
    size_t num_joint_dof = 0;
    size_t num_joint_coord = 0;
    std::vector<float> joint_q;     // Generalized joint positions for state initialization
    std::vector<float> joint_qd;    // Generalized joint velocities for state initialization
    std::vector<float> joint_f;     // Generalized joint forces for state initialization
    std::vector<JointType> joint_type;  // Joint type
    std::vector<std::string> joint_key; // Joint keys

    std::vector<int> joint_parent;      // Joint parent body indices
    std::vector<int> joint_child;       // Joint child body indices
    std::vector<int> joint_articulation;   // Joint articulation index (-1 if not in any articulation)

    std::vector<TTransform> joint_X_p;  // Joint transform in parent frame
    std::vector<TTransform> joint_X_c;  // Joint mass frame in child frame
    std::vector<int> joint_q_start;     // Start index of the first position coordinate per joint (last value is a sentinel for dimension queries)
    std::vector<int> joint_qd_start;    // Start index of the first velocity coordinate per joint (last value is a sentinel for dimension queries)
    std::vector<std::pair<int, int>> joint_dof_dim;     // Number of linear and angular dofs per joint

    std::vector<float> joint_target_pos;    // Generalized joint position targets, shape [joint_dof_count]
    std::vector<float> joint_target_vel;    // Generalized joint velocity targets
    std::vector<Vec3> joint_axis;           // Joint axis in child frame, shape [joint_dof_count, 3]
    std::vector<float> joint_target_ke;     // Joint stiffness
    std::vector<float> joint_target_kd;     // Joint damping, shape [joint_dof_count], float
    std::vector<float> joint_limit_ke;      // Joint position limit stiffness
    std::vector<float> joint_limit_kd;      // Joint position limit damping
    std::vector<float> joint_armature;      // Armature for each joint axis
    std::vector<float> joint_effort_limit;  // Joint effort (force/torque) limits
    std::vector<float> joint_velocity_limit;    // Joint velocity limits
    std::vector<float> joint_friction;          // Joint friction coefficient
    std::vector<float> joint_limit_lower;       // Joint lower position limits,
    std::vector<float> joint_limit_upper;       // Joint upper position limits

    // ------ articulation ------
    size_t num_articulation = 0;
    std::vector<int> articulation_start;        // Articulation start index
    std::vector<std::string> articulation_key;          // Articulation keys


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
    CollideParams collide_params{};
    std::unique_ptr<CollisionPipeline> collision_pipeline_;

    const Contacts* collide(const State& state_in) {
        if (!collision_pipeline_) {
            collision_pipeline_ = std::make_unique<CollisionPipeline>();
            collision_pipeline_->BuildFromModel(*this, collide_params);
        }
        const auto contacts = collision_pipeline_->Collide(*this, state_in);
        return contacts;
    };

};



// void ParseMSH(const std::string& path, mesh_on_cpu* cpu_mesh);

// IndexBuffer BuildSurfaceTriangles(const std::vector<tetrahedron>& tets);

// IndexBuffer BuildSurfaceTriangles(const std::vector<triangle>& tris);



#endif //TAIYI_MODEL_H
