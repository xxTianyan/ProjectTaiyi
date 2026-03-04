//
// Created by xumiz on 2026/1/2.
//

#include "SampleRegistry.h"
#include "VBDSolver.h"
#include "Sample.h"
#include "hanging_cloth.hpp"
#include "falling_bunny.hpp"
#include "falling_cloth.hpp"
#include "rigid_box.hpp"
#include "rubber_ball.hpp"
#include "basic_joints.hpp"
#include "debug_scene.hpp"
#include "humanoid.hpp"
#include "debug_scene.hpp"

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

    reg.Register(SampleId::RIGID_BOX, "Rigid Box",
        []()->SamplePtr {return std::make_unique<RigidBox>();});

    reg.Register(SampleId::BASIC_JOINTS, "Basic Joints",
        []()->SamplePtr {return std::make_unique<BasicJoints>();});

    reg.Register(SampleId::HUMANOID, "Humanoid",
        []()->SamplePtr {return std::make_unique<Humanoid>();});

    reg.Register(SampleId::DEBUG, "Debug Lab",
        []()->SamplePtr {return std::make_unique<DebugLab>();});
}
