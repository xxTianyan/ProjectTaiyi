#include "tiny_vbd/solver/Solver.h"

#include "tiny_vbd/collision/Collision.h"
#include "tiny_vbd/contact/Contact.h"

namespace tinyvbd {

State Solver::createState(const Model& model) {
    State state;
    const std::size_t particle_count = model.particles.size();

    state.positions.resize(particle_count);
    state.velocities.assign(particle_count, Vec3::Zero());
    state.external_forces.assign(particle_count, Vec3::Zero());

    for (std::size_t i = 0; i < particle_count; ++i) {
        state.positions[i] = model.particles[i].rest_position;
    }

    state.distance_lambdas.assign(model.distance_constraints.size(), 0.0f);
    state.render_mesh.render_positions = state.positions;
    state.render_mesh.render_indices = model.render_indices;
    return state;
}

void Solver::step(const Model& model,
                  State& state,
                  const float dt,
                  const int substeps,
                  const int iterations,
                  const Vec3& gravity) {
    const float h = dt / static_cast<float>(substeps);

    for (int s = 0; s < substeps; ++s) {
        std::vector<Vec3> predicted = state.positions;

        for (std::size_t i = 0; i < model.particles.size(); ++i) {
            const float inv_mass = model.particles[i].inv_mass;
            if (inv_mass <= 0.0f) {
                continue;
            }
            const Vec3 accel = gravity + state.external_forces[i] * inv_mass;
            state.velocities[i] += h * accel;
            predicted[i] += h * state.velocities[i];
        }

        std::fill(state.distance_lambdas.begin(), state.distance_lambdas.end(), 0.0f);
        for (int iter = 0; iter < iterations; ++iter) {
            solveDistanceConstraints(model, predicted, state.distance_lambdas, h);
            auto contacts = detectPlaneContacts(model, predicted);
            solveContacts(model, predicted, contacts);
        }

        for (std::size_t i = 0; i < model.particles.size(); ++i) {
            const float inv_mass = model.particles[i].inv_mass;
            if (inv_mass <= 0.0f) {
                state.velocities[i] = Vec3::Zero();
                predicted[i] = state.positions[i];
                continue;
            }
            state.velocities[i] = (predicted[i] - state.positions[i]) / h;
            state.positions[i] = predicted[i];
        }
    }

    state.render_mesh.render_positions = state.positions;
}

} // namespace tinyvbd
