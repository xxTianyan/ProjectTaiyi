#pragma once

#include <vector>

#include "tiny_vbd/collision/Collision.h"
#include "tiny_vbd/model/Model.h"

namespace tinyvbd {

void solveDistanceConstraints(const Model& model,
                              std::vector<Vec3>& predicted_positions,
                              std::vector<float>& lambdas,
                              float dt);

void solveContacts(const Model& model,
                   std::vector<Vec3>& predicted_positions,
                   const std::vector<PlaneContact>& contacts);

} // namespace tinyvbd
