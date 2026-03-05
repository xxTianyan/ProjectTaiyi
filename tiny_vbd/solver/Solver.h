#pragma once

#include "tiny_vbd/model/Model.h"
#include "tiny_vbd/state/State.h"

namespace tinyvbd {

class Solver {
public:
    static State createState(const Model& model);

    static void step(const Model& model,
                     State& state,
                     float dt,
                     int substeps,
                     int iterations,
                     const Vec3& gravity);
};

} // namespace tinyvbd
