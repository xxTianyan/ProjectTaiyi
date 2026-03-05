#include "tiny_vbd/builder/Builder.h"

#include <stdexcept>

namespace tinyvbd {

namespace {
std::uint32_t particleIndex(const int r, const int c, const int cols) {
    return static_cast<std::uint32_t>(r * cols + c);
}
}

int Builder::addClothPatch(const ClothPatchDesc& desc) {
    if (desc.rows < 2 || desc.cols < 2) {
        throw std::invalid_argument("cloth rows/cols must be >= 2");
    }

    const std::size_t particle_offset = model_.particles.size();
    const std::size_t vertex_offset = model_.particles.size();
    const std::size_t index_offset = model_.render_indices.size();

    const float inv_mass = desc.particle_mass > 0.0f ? 1.0f / desc.particle_mass : 0.0f;

    for (int r = 0; r < desc.rows; ++r) {
        for (int c = 0; c < desc.cols; ++c) {
            ParticleDesc p;
            p.rest_position = desc.origin + Vec3(desc.spacing * static_cast<float>(c),
                                                 0.0f,
                                                 desc.spacing * static_cast<float>(r));
            const bool pinned = desc.pin_top_row && r == 0;
            p.inv_mass = pinned ? 0.0f : inv_mass;
            model_.particles.push_back(p);
        }
    }

    auto add_distance = [&](int r0, int c0, int r1, int c1) {
        const auto i_local = particleIndex(r0, c0, desc.cols);
        const auto j_local = particleIndex(r1, c1, desc.cols);
        const auto i = static_cast<std::uint32_t>(particle_offset + i_local);
        const auto j = static_cast<std::uint32_t>(particle_offset + j_local);

        DistanceConstraint cons;
        cons.i = i;
        cons.j = j;
        const Vec3 pi = model_.particles[i].rest_position;
        const Vec3 pj = model_.particles[j].rest_position;
        cons.rest_length = (pi - pj).norm();
        cons.compliance = desc.distance_compliance;
        model_.distance_constraints.push_back(cons);
    };

    for (int r = 0; r < desc.rows; ++r) {
        for (int c = 0; c < desc.cols; ++c) {
            if (c + 1 < desc.cols) {
                add_distance(r, c, r, c + 1);
            }
            if (r + 1 < desc.rows) {
                add_distance(r, c, r + 1, c);
            }
            if (r + 1 < desc.rows && c + 1 < desc.cols) {
                add_distance(r, c, r + 1, c + 1);
            }
            if (r + 1 < desc.rows && c - 1 >= 0) {
                add_distance(r, c, r + 1, c - 1);
            }
        }
    }

    for (int r = 0; r < desc.rows - 1; ++r) {
        for (int c = 0; c < desc.cols - 1; ++c) {
            const std::uint32_t v00 = static_cast<std::uint32_t>(vertex_offset + particleIndex(r, c, desc.cols));
            const std::uint32_t v10 = static_cast<std::uint32_t>(vertex_offset + particleIndex(r + 1, c, desc.cols));
            const std::uint32_t v01 = static_cast<std::uint32_t>(vertex_offset + particleIndex(r, c + 1, desc.cols));
            const std::uint32_t v11 = static_cast<std::uint32_t>(vertex_offset + particleIndex(r + 1, c + 1, desc.cols));

            model_.render_indices.push_back(v00);
            model_.render_indices.push_back(v10);
            model_.render_indices.push_back(v01);

            model_.render_indices.push_back(v01);
            model_.render_indices.push_back(v10);
            model_.render_indices.push_back(v11);
        }
    }

    ObjectRange object;
    object.particle_offset = particle_offset;
    object.particle_count = static_cast<std::size_t>(desc.rows * desc.cols);
    object.render_range.vertex_offset = vertex_offset;
    object.render_range.vertex_count = object.particle_count;
    object.render_range.index_offset = index_offset;
    object.render_range.index_count = model_.render_indices.size() - index_offset;
    object.render_range.object_id = static_cast<int>(model_.objects.size());

    model_.objects.push_back(object);
    return object.render_range.object_id;
}

int Builder::addPlaneCollider(const PlaneColliderDesc& desc) {
    model_.plane_colliders.push_back(PlaneCollider{desc.normal.normalized(), desc.offset});
    return static_cast<int>(model_.plane_colliders.size() - 1);
}

Model Builder::build() const {
    return model_;
}

} // namespace tinyvbd
