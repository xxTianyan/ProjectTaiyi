#include "tiny_vbd/contact/Contact.h"

#include <algorithm>

namespace tinyvbd {

void solveDistanceConstraints(const Model& model,
                              std::vector<Vec3>& predicted_positions,
                              std::vector<float>& lambdas,
                              const float dt) {
    const float dt2 = dt * dt;
    for (std::size_t c = 0; c < model.distance_constraints.size(); ++c) {
        const DistanceConstraint& cons = model.distance_constraints[c];
        Vec3& xi = predicted_positions[cons.i];
        Vec3& xj = predicted_positions[cons.j];

        const float wi = model.particles[cons.i].inv_mass;
        const float wj = model.particles[cons.j].inv_mass;
        if (wi + wj <= 0.0f) {
            continue;
        }

        Vec3 d = xi - xj;
        const float len = d.norm();
        if (len <= 1.0e-7f) {
            continue;
        }

        const Vec3 n = d / len;
        const float C = len - cons.rest_length;
        const float alpha = cons.compliance / dt2;
        const float denom = wi + wj + alpha;

        float& lambda = lambdas[c];
        const float delta_lambda = (-C - alpha * lambda) / denom;
        lambda += delta_lambda;

        xi += wi * delta_lambda * n;
        xj -= wj * delta_lambda * n;
    }
}

void solveContacts(const Model& model,
                   std::vector<Vec3>& predicted_positions,
                   const std::vector<PlaneContact>& contacts) {
    for (const PlaneContact& contact : contacts) {
        const float inv_mass = model.particles[contact.particle].inv_mass;
        if (inv_mass <= 0.0f) {
            continue;
        }
        predicted_positions[contact.particle] += contact.penetration * contact.normal;
    }
}

} // namespace tinyvbd
