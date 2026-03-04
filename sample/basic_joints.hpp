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
#include "Order/OrderSolver.h"

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

        sphere_ = builder.add_rigidbody("sphere1", Vec3{0.0f,5.0f,0.0f}, Quat::Identity());
        auto sphere_shape = builder.add_shape_sphere(sphere_, 0.4, Vec3::Zero(), Quat::Identity());

        capsule_ = builder.add_rigidbody("capsule1", Vec3{0.0f,3.9f,0.0f}, Quat{1.0,0.0,0.0,0.0});
        auto capsule_shape = builder.add_shape_capsule(capsule_, 0.2, 0.5);


        const auto joint_fix = builder.add_joint_fixed(
            -1,
            sphere_,
            TTransform(Vec3(0.0, 5.0, 0.0), Quat::Identity()),
            TTransform::Identity(),
            "free_joint");

        const auto joint_revolute = builder.add_joint_revolute(
            static_cast<int>(sphere_),
            static_cast<int>(capsule_),
            Vec3::UnitZ(),
            TTransform(Vec3(0.0, -0.4, 0.0), Quat::Identity()),
            TTransform(Vec3(0.0,0.7,0.0), Quat::Identity()));


        auto art_0 = builder.add_articulation(std::array{joint_fix, joint_revolute}, "sphere_n_capsule");

        auto& revolute_degree = model.joint_q0.back();
        revolute_degree = 3.1415926f * 0.25f;

        builder.finalize();

        scene_ = std::make_unique<Scene>(std::move(model));
        dbg_ = std::make_unique<SolverDebugger>();
        solver_ = std::make_unique<OrderSolver>(scene_->model_);
    }


    void BindShaders(AppContext &ctx) override {
        ctx.shader_manager->LoadShaderProgram("rubber", "../resources/shaders/rubber.vs", "../resources/shaders/rubber.fs");
        const auto rubber_shader = ctx.shader_manager->Get("rubber")->shader;
        ShaderManager::BindMatrices(rubber_shader);
        ShaderManager::SetCommonShaderParams(rubber_shader);
        rubber_shader.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(rubber_shader, "texture0");

        auto sphere_model = renderHelper_.GetRLModel_r(sphere_);
        sphere_model.materials[0].shader = rubber_shader;
        sphere_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};

        auto capsule_model = renderHelper_.GetRLModel_r(capsule_);
        capsule_model.materials[0].shader = rubber_shader;
        capsule_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};
    }

private:
    size_t sphere_{};
    size_t capsule_{};


    size_t shape_ground_plane_{};
};



#endif //TAIYI_BASIC_JOINTS_HPP