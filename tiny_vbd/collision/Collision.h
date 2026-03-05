#pragma once

#include <vector>

#include "tiny_vbd/model/Model.h"

namespace tinyvbd {

struct PlaneContact {
    std::uint32_t particle = 0;
    Vec3 normal = Vec3(0.0f, 1.0f, 0.0f);
    float penetration = 0.0f;
};

std::vector<PlaneContact> detectPlaneContacts(const Model& model, const std::vector<Vec3>& predicted_positions);

} // namespace tinyvbd
