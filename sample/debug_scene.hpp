//
// Created by tianyan on 3/4/26.
//

#ifndef TAIYI_DEBUG_SCENE_HPP
#define TAIYI_DEBUG_SCENE_HPP
#include "Builder.h"
#include "ISolver.h"
#include "MaterialParams.hpp"
#include "Sample.h"
#include "Scene.h"
#include "Order/OrderSolver.h"

class DebugLab final : public Sample {

public:
    DebugLab() {
        max_ticks_per_frame0_ = 2;
        substeps0_ = 12;
    }

    void Step(const float dt) override {

        if (scene_ == nullptr) return;
        if (solver_ == nullptr) return;

        // collide here
        const Contacts* contacts = scene_->model_.collide(scene_->state_out());

        solver_->Step(scene_->state_in(), scene_->state_out(), contacts, dt);
        scene_->SwapStates();
    };

    void CreateWorld([[maybe_unused]]AppContext &ctx) override {
        MModel model;
        Builder builder(model);
        shape_ground_plane_ = builder.add_ground_plane();

        // ----------------------- sphere -----------------------
        /*int b1 = builder.add_rigidbody("sphere1", Vec3{0.0f,5.0f,0.0f}, Quat::Identity());
        auto shape_ = builder.add_shape_sphere(b1, 0.4, Vec3::Zero(), Quat::Identity());
        bodies_.push_back(b1);

        const int joint = builder.add_joint_free(b1,
            TTransform::Identity(),
            TTransform::Identity(),
            "joint_free1");

        model.shape_contact_pairs.emplace_back(shape_, shape_ground_plane_);

        const int art_ = builder.add_articulation(std::array{joint},"art_1");*/

        // ---------------------- capsule ------------------------
        int b2 = builder.add_link("capsule1", Vec3{1.0f,7.0f,1.0f}, Quat{0.9238795325,0.3826834324,0,0});
        auto shape_2 = builder.add_shape_capsule(b2, 0.2, 0.5);
        bodies_.push_back(b2);

        const int joint2 = builder.add_joint_free(b2,
            TTransform::Identity(),
            TTransform::Identity(),
            "joint_free2");

        model.shape_contact_pairs.emplace_back(shape_2, shape_ground_plane_);
        const int art2_ = builder.add_articulation(std::array{joint2}, "art_2");

        builder.finalize();

        scene_ = std::make_unique<Scene>(std::move(model));
        dbg_ = std::make_unique<SolverDebugger>();
        solver_ = std::make_unique<OrderSolver>(scene_->model_);
    };

    void BindShaders(AppContext &ctx) override {
        ctx.shader_manager->LoadShaderProgram("rubber", "../resources/shaders/rubber.vs", "../resources/shaders/rubber.fs");
        const auto rubber_shader = ctx.shader_manager->Get("rubber")->shader;
        ShaderManager::BindMatrices(rubber_shader);
        ShaderManager::SetCommonShaderParams(rubber_shader);
        rubber_shader.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(rubber_shader, "texture0");

        for (const int b : bodies_) {
            auto body_model = renderHelper_.GetRLModel_r(b);
            body_model.materials[0].shader = rubber_shader;
            body_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};
        }

    }


private:
    std::vector<int> bodies_;
    size_t shape_ground_plane_{};

};






#endif //TAIYI_DEBUG_SCENE_HPP