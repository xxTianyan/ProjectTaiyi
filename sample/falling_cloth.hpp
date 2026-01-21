//
// Created by tianyan on 1/21/26.
//

#ifndef TAIYI_FOLDING_CLOTH_HPP
#define TAIYI_FOLDING_CLOTH_HPP

#include "Sample.h"
#include "Application.h"
#include "Builder.h"
#include "rlgl.h"
#include "Scene.h"
#include "VBDSolver.h"
#include "TriMeshCollision.h"


class FallingCloth final : public Sample {

public:

    FallingCloth() {
        max_ticks_per_frame0_ = 1;
        substeps0_ = 4;
    }

    void CreateWorld([[maybe_unused]]AppContext &ctx) override {
        MModel model;
        Builder builder(model);
        m_cloth_id_ = builder.add_cloth(2.0f, 3.0f, 12, 18, Vec3{0.0f, 4.0f, 0.0f}, 0.1, ClothOrientation::Vertical, NONE);
        scene_ = std::make_unique<Scene>(std::move(model));
        dbg_ = std::make_unique<SolverDebugger>();
        solver_ = std::make_unique<VBDSolver>(scene_->model_, 3, default_cloth(),dbg_.get());
        solver_->set_self_collision(0.16, 0.7, 0.43);

    };

    void Render(AppContext &ctx) override {

        AABBTreeDrawSettings dbgTriTree;
        dbgTriTree.enabled = true;
        dbgTriTree.leavesOnly = false;   // 先画 internal + leaf 看整体分割
        dbgTriTree.maxDepth = -1;        // 不要太大


        BeginMode3D(ctx.orbitCam->camera);

        // floor
        if (RenderHelper::IsModelValid(floor_)) {
            DrawModel(floor_, Vector3{0,0,0}, 1.0f, WHITE);
        }

        // scene models
        rlDisableBackfaceCulling();
        renderHelper_.Draw(ctx.is_wire_mode);
        rlEnableBackfaceCulling();

        solver_->draw_triangle_bvh(dbgTriTree);

        EndMode3D();
    };

    void BindShaders(AppContext &ctx) override {
        ctx.shader_manager->LoadShaderProgram("cloth", "../resources/shaders/cloth.vs", "../resources/shaders/cloth.fs");
        const auto cloth_shader = ctx.shader_manager->Get("cloth")->shader;
        ShaderManager::BindMatrices(cloth_shader);
        ShaderManager::SetCommonShaderParams(cloth_shader);
        cloth_shader.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(cloth_shader, "texture0");
        auto m_cloth_model = renderHelper_.GetRLModel(m_cloth_id_);
        m_cloth_model.materials[0].shader = cloth_shader;
        m_cloth_model.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};
        // m_cloth.materials[0].maps[MATERIAL_MAP_ALBEDO].texture = clothAlbedoTex;          // if texture

        const int rough = ShaderManager::CheckSetShaderLocation(cloth_shader, "roughness");
        const int specStr = ShaderManager::CheckSetShaderLocation(cloth_shader, "specStrength");
        const int wrap = ShaderManager::CheckSetShaderLocation(cloth_shader, "wrapDiffuse");

        constexpr float clothRoughness = 0.80f;  // 越大越哑、highlight 越宽
        constexpr float clothSpec      = 0.22f;  // 高光强度：太大像塑料，太小没质感
        constexpr float clothWrap      = 0.25f;  // 漫反射包裹：增大可让暗面不至于太死
        SetShaderValue(cloth_shader, rough,   &clothRoughness, SHADER_UNIFORM_FLOAT);
        SetShaderValue(cloth_shader, specStr, &clothSpec,      SHADER_UNIFORM_FLOAT);
        SetShaderValue(cloth_shader, wrap,    &clothWrap,      SHADER_UNIFORM_FLOAT);
    };


private:
    size_t m_cloth_id_{};

};





#endif //TAIYI_FOLDING_CLOTH_HPP