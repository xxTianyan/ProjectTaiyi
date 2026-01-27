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
        m_box_id = builder.add_rigidbox(Vec3(0, 1, 0), Vec3(0.5, 0.2, 0.5), 1.0f, Quat::Identity(),"test_box");
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
        auto m_model = renderHelper_.GetRLModel_r(m_box_id);
        m_model.materials[0].shader = bunny_shader;
        m_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};
    };

private:
    size_t m_box_id{};
};



#endif //TAIYI_RIGID_BOX_HPP