//
// Created by tianyan on 12/22/25.
//

#include "VBDSolver.h"
#include "Debugger.hpp"

inline void AssignOffsets(const size_t num_nodes, std::vector<uint32_t>& offsets) {
    offsets.assign(num_nodes + 1, 0u);
}

void VBDSolver::clear() {

    if (model_.topology_version != topology_version_) {
        BuildAdjacencyInfo();
        topology_version_ = model_.topology_version;

        // record surface vertex
        surface_vertices.resize(model_.render_tris.size()*3, 0);
        for (const auto& tri: model_.render_tris) {
            surface_vertices[tri.vertices[0]] = 1;
            surface_vertices[tri.vertices[1]] = 1;
            surface_vertices[tri.vertices[2]] = 1;
        }

        // resize vectors
        const size_t num_nodes = model_.num_particles;
        particle_inertia_.resize(num_nodes);
        particle_prev_pos_.resize(num_nodes);
        particle_contact_force_.resize(num_nodes);
        particle_contact_hessian_.resize(num_nodes);

        const size_t num_bodies = model_.num_bodies;
        body_prev_pos_.resize(num_bodies);
        body_prev_rot_.resize(num_bodies);
        body_inertia_pos_.resize(num_bodies);
        body_inertia_rot_.resize(num_bodies);
        body_force_.resize(num_bodies);
        body_torque_.resize(num_bodies);
        body_hessian_aa_.resize(num_bodies);
        body_hessian_al_.resize(num_bodies);
        body_hessian_ll_.resize(num_bodies);

        body_body_contact_counts_.resize(num_bodies);
        body_body_contact_counts_indices_.resize(num_bodies * num_pre_alloc_contacts, -1);
    }

    std::ranges::fill(particle_contact_force_, Vec3::Zero());
    std::ranges::fill(particle_contact_hessian_, Mat3::Zero());

    std::ranges::fill(body_force_, Vec3::Zero());
    std::ranges::fill(body_torque_, Vec3::Zero());
    std::ranges::fill(body_hessian_aa_, Mat3::Zero());
    std::ranges::fill(body_hessian_al_, Mat3::Zero());
    std::ranges::fill(body_hessian_ll_, Mat3::Zero());
}

void VBDSolver::Step(State& state_in, State& state_out, const Contacts* contacts, const float dt) {

    if (dt <= 0.0f) {
        return;
    }

    clear();

    init_rigid_bodies(state_in, contacts, dt);
    init_particles(state_in, dt);


    for (int iter = 0; iter < num_iters; ++iter) {
        ScopeTimer iter_timer = dbg_ ? dbg_->timer_iteration() : ScopeTimer(nullptr);
        solve_rigid_body(state_in, state_out, contacts, dt);
        solve_particle(state_in, state_out, dt);
    }

    update_rigid_body_vel(state_out, model_.body_local_com, dt);
    update_particle_vel(state_out, dt);

}

void VBDSolver::set_self_collision(const float particle_contact_margin, const float particle_rest_shape_contact_exclusion_radius,
                                   const float conservative_bound_relaxation) {
    if (detector_) return;
    detector_ = std::make_unique<TriMeshCollisionDetector>(model_, adjacency_info_);
    detector_->particle_contact_margin = particle_contact_margin;
    detector_->particle_rest_shape_contact_exclusion_radius = particle_rest_shape_contact_exclusion_radius;
    detector_->conservative_bound_relaxation = conservative_bound_relaxation;
}

void VBDSolver::BuildAdjacencyInfo() {
    const size_t num_nodes = model_.num_particles;
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











