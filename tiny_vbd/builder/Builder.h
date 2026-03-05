#pragma once

#include "tiny_vbd/model/Model.h"

namespace tinyvbd {

struct ClothPatchDesc {
    int rows = 2;
    int cols = 2;
    float spacing = 0.1f;
    Vec3 origin = Vec3(0.0f, 1.0f, 0.0f);
    float particle_mass = 1.0f;
    float distance_compliance = 1.0e-6f;
    bool pin_top_row = true;
};

struct PlaneColliderDesc {
    Vec3 normal = Vec3(0.0f, 1.0f, 0.0f);
    float offset = 0.0f;
};

class Builder {
public:
    int addClothPatch(const ClothPatchDesc& desc);
    int addPlaneCollider(const PlaneColliderDesc& desc);

    [[nodiscard]] Model build() const;

private:
    Model model_;
};

} // namespace tinyvbd
