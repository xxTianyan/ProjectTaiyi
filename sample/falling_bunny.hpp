//
// Created by tianyan on 1/9/26.
//

#ifndef TAIYI_BUNNY_H
#define TAIYI_BUNNY_H

#include "Sample.h"
#include "falling_bunny.hpp"
#include <iostream>
#include "Application.h"
#include "Builder.h"
#include "Scene.h"
#include "ShaderManager.h"
#include "VBDSolver.h"


class FallingBunny final : public Sample {

public:
    FallingBunny() {
        max_ticks_per_frame_ = 8;
        substeps_ = 16;
    };
    void CreateWorld([[maybe_unused]]AppContext &ctx) override {
        MModel model;
        Builder builder(model);
        m_bunny_id_ = builder.add_bunny(3.0, 0.5);
        scene_ = std::make_unique<Scene>(std::move(model));
        dbg_ = std::make_unique<SolverDebugger>();
        solver_ = std::make_unique<VBDSolver>(scene_->model_, 4, soft_bunny(), dbg_.get());

    };
    void BindShaders(AppContext &ctx) override {
        ctx.shader_manager->LoadShaderProgram("rubber", "../resources/shaders/rubber.vs", "../resources/shaders/rubber.fs");
        const auto bunny_shader = ctx.shader_manager->Get("rubber")->shader;
        ShaderManager::BindMatrices(bunny_shader);
        ShaderManager::SetCommonShaderParams(bunny_shader);
        bunny_shader.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(bunny_shader, "texture0");
        auto m_bunny_model = renderHelper_.GetRLModel(m_bunny_id_);
        m_bunny_model.materials[0].shader = bunny_shader;
        m_bunny_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};
    };

private:
    size_t m_bunny_id_{};
};



#endif //TAIYI_BUNNY_H