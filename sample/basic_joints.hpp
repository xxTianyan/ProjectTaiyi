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

    BasicJoints() {
        max_ticks_per_frame0_ = 2;
        substeps0_ = 12;
    }

    void Step(float dt) override {
        if (scene_ == nullptr) return;
        if (solver_ == nullptr) return;

        // collide here
        const Contacts* contacts = scene_->model_.collide(scene_->state_out());

        solver_->Step(scene_->state_in(), scene_->state_out(), contacts, dt);
        scene_->SwapStates();
    };

    void CreateWorld([[maybe_unused]]AppContext& ctx) override {
        MModel model;
        Builder builder(model);
        shape_ground_plane_ = builder.add_ground_plane();

        sphere_ = builder.add_rigidbody("sphere1", Vec3{0.0f,3.0f,0.0f}, Quat{1.0f,0.0f,0.0f,0.0f});
        auto sphere_shape = builder.add_shape_sphere(sphere_, 0.4);

        capsule_ = builder.add_rigidbody("capsule1", Vec3{0.0f,2.3f,0.0f}, Quat{1.0,0.0,0.0,0.0});
        auto capsule_shape = builder.add_shape_capsule(capsule_, 0.2, 0.5);

        // add collide pair. temporary
        model.shape_contact_pairs.emplace_back(sphere_shape, shape_ground_plane_);
        model.shape_contact_pairs.emplace_back(capsule_shape, shape_ground_plane_);

        const auto joint_free = builder.add_joint_free(
            sphere_,
            TTransform(Vec3{0.0f, 3.0f, 0.0f}, Quat{1.0f, 0.0f, 0.0f, 0.0f}),
            TTransform::Identity(),
            "free_joint");

        const auto joint_revolute = builder.add_joint_revolute(
            static_cast<int>(sphere_),
            static_cast<int>(capsule_),
            Vec3::UnitX(),
            TTransform::Identity(),
            TTransform(Vec3(0.0,0.7,0.0), Quat::Identity()));

        builder.add_articulation(std::array{joint_free, joint_revolute}, "sphere_n_capsule");

        scene_ = std::make_unique<Scene>(std::move(model));
        dbg_ = std::make_unique<SolverDebugger>();
        solver_ = std::make_unique<VBDSolver>(scene_->model_, 10, soft_bunny(), dbg_.get());



    }



private:
    size_t sphere_{};
    size_t capsule_{};
    size_t shape_ground_plane_{};
};



#endif //TAIYI_BASIC_JOINTS_HPP