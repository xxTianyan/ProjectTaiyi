#pragma once

#include <vector>

#include "common/Types.h"

namespace tinyvbd {

struct RenderMesh {
    std::vector<Vec3> render_positions;
    std::vector<std::uint32_t> render_indices;
};

struct State {
    std::vector<Vec3> positions;
    std::vector<Vec3> velocities;
    std::vector<Vec3> external_forces;
    std::vector<float> distance_lambdas;

    RenderMesh render_mesh;
};

} // namespace tinyvbd
