#include "tiny_vbd/collision/Collision.h"

namespace tinyvbd {

std::vector<PlaneContact> detectPlaneContacts(const Model& model, const std::vector<Vec3>& predicted_positions) {
    std::vector<PlaneContact> contacts;
    contacts.reserve(predicted_positions.size());

    for (std::uint32_t i = 0; i < predicted_positions.size(); ++i) {
        const Vec3& x = predicted_positions[i];
        for (const PlaneCollider& plane : model.plane_colliders) {
            const float signed_distance = plane.normal.dot(x) + plane.offset;
            if (signed_distance < 0.0f) {
                contacts.push_back(PlaneContact{i, plane.normal, -signed_distance});
            }
        }
    }

    return contacts;
}

} // namespace tinyvbd
