//
// Created by tianyan on 1/26/26.
//

#ifndef TAIYI_RIGID_BOX_HPP
#define TAIYI_RIGID_BOX_HPP



#include "Sample.h"
#include "Application.h"
#include "Builder.h"
#include "Scene.h"
#include "ShaderManager.h"
#include "VBDSolver.h"


class RigidBox final : public Sample {

public:
    RigidBox() {
        max_ticks_per_frame0_ = 8;
        substeps0_ = 8;
    };

    void CreateWorld([[maybe_unused]]AppContext &ctx) override {
        MModel model;
        Builder builder(model);

        rigid_box_ = builder.add_rigidbody("box", Vec3{4.0f,5.0f,0.0f}, Quat::Identity());
        auto box_shape = builder.add_shape_box(rigid_box_, 1.0f, 1.0f, 1.0f);

        bunny_ = builder.add_bunny(8.0, 0.5);

        scene_ = std::make_unique<Scene>(std::move(model));
        dbg_ = std::make_unique<SolverDebugger>();
        solver_ = std::make_unique<VBDSolver>(scene_->model_, 2, soft_bunny(), dbg_.get());
    };

    void BindShaders(AppContext &ctx) override {
        ctx.shader_manager->LoadShaderProgram("rubber", "../resources/shaders/rubber.vs", "../resources/shaders/rubber.fs");
        const auto bunny_shader = ctx.shader_manager->Get("rubber")->shader;
        ShaderManager::BindMatrices(bunny_shader);
        ShaderManager::SetCommonShaderParams(bunny_shader);
        bunny_shader.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(bunny_shader, "texture0");

        auto box_model = renderHelper_.GetRLModel_r(rigid_box_);
        box_model.materials[0].shader = bunny_shader;
        box_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};

        auto bunny_model = renderHelper_.GetRLModel_d(bunny_);
        bunny_model.materials[0].shader = bunny_shader;
        bunny_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};
    };

private:
    size_t rigid_box_{};
    size_t bunny_{};
};



#endif //TAIYI_RIGID_BOX_HPP