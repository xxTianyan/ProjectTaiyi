//
// Created by xumiz on 2026/1/2.
//

#include "SampleRegistry.h"
#include "VBDSolver.h"
#include "Sample.h"
#include "hanging_cloth.hpp"
#include "falling_bunny.hpp"
#include "falling_cloth.hpp"
#include "rubber_ball.hpp"

void RegisterAllSamples(SampleRegistry& reg) {

    // Empty Scene
    reg.Register(SampleId::EMPTY_SCENE, "Empty Scene",
        []() -> SamplePtr { return std::make_unique<Sample>(); });

    reg.Register(SampleId::HANGING_CLOTH, "Hanging Cloth",
        []()->SamplePtr { return std::make_unique<HangingCloth>();});

    reg.Register(SampleId::FALLING_BUNNY, "Falling Bunny",
    []()->SamplePtr { return std::make_unique<FallingBunny>();});

    reg.Register(SampleId::RUBBER_BALL, "Rubber Ball",
        []()->SamplePtr {return std::make_unique<RubberBall>();});

    reg.Register(SampleId::FALLING_CLOTH, "Falling Cloth",
        []()->SamplePtr {return std::make_unique<FallingCloth>();});
}
