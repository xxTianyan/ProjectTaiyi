#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "common/Types.h"

namespace tinyvbd {

struct RenderRange {
    std::size_t vertex_offset = 0;
    std::size_t vertex_count = 0;
    std::size_t index_offset = 0;
    std::size_t index_count = 0;
    int object_id = -1;
    int material_id = 0;
};

struct ParticleDesc {
    Vec3 rest_position = Vec3::Zero();
    float inv_mass = 1.0f;
};

struct DistanceConstraint {
    std::uint32_t i = 0;
    std::uint32_t j = 0;
    float rest_length = 0.0f;
    float compliance = 0.0f;
};

struct PlaneCollider {
    Vec3 normal = Vec3(0.0f, 1.0f, 0.0f);
    float offset = 0.0f;
};

struct ObjectRange {
    std::size_t particle_offset = 0;
    std::size_t particle_count = 0;
    RenderRange render_range;
};

struct Model {
    std::vector<ParticleDesc> particles;
    std::vector<DistanceConstraint> distance_constraints;
    std::vector<PlaneCollider> plane_colliders;

    std::vector<std::uint32_t> render_indices;
    std::vector<ObjectRange> objects;
};

} // namespace tinyvbd
