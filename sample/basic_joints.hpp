//
// Created by tianyan on 2/25/26.
//

#ifndef TAIYI_BASIC_JOINTS_HPP
#define TAIYI_BASIC_JOINTS_HPP
#include "Builder.h"
#include "MaterialParams.hpp"
#include "Sample.h"
#include "Scene.h"
#include "VBDSolver.h"

class BasicJoints final : public Sample {

public:

    void CreateWorld([[maybe_unused]]AppContext& ctx) override {
        MModel model;
        Builder builder(model);
        shape_ground_plane_ = builder.add_ground_plane();

        sphere_ = builder.add_rigidbody("sphere1", Vec3{0.0f,3.0f,0.0f}, Quat{1.0f,0.0f,0.0f,0.0f});
        auto sphere_shape = builder.add_shape_sphere(sphere_, 0.2);

        scene_ = std::make_unique<Scene>(std::move(model));
        dbg_ = std::make_unique<SolverDebugger>();
        solver_ = std::make_unique<VBDSolver>(scene_->model_, 10, soft_bunny(), dbg_.get());

    }

private:
    size_t sphere_{};
    size_t shape_ground_plane_{};
};



#endif //TAIYI_BASIC_JOINTS_HPP