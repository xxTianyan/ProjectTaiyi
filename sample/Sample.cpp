//
// Created by xumiz on 2025/12/24.
//

#include "Sample.h"
#include "Application.h"
#include "VBDSolver.h"

void Sample::OnEnter(AppContext &ctx) {
    // set up scene
    CreateWorld(ctx);

    // set up floor
    CreateFloor(ctx);

    // upload mesh to gpu
    BuildRenderResources();

    // init and bind shaders
    BindShaders(ctx);

}

void Sample::OnExit([[maybe_unused]]AppContext &ctx) {
    // clean cpu resource
    CleanUp();

    // clean gpu resource, important!!
    DestroyRenderResources();
}

void Sample::Update([[maybe_unused]]AppContext &ctx) {

    if (ctx.paused) return;
    if (scene_ == nullptr) return;

    if (dbg_ && !dbg_->begin_step(ctx.frame_id))
        return;

    // physical frame
    {
        ScopeTimer frame_timer = dbg_ ? dbg_->timer_frame() : ScopeTimer(nullptr);
        scene_->InitStep();

        // accumulate simulation time
        float frame_dt = ctx.dt;
        if (frame_dt > 0.05f) frame_dt = 0.05f; // prevent dt explosion
        sim_accum_ += frame_dt;

        //run simulation time
        int ticks = 0;
        while (sim_accum_ >= fixed_dt_ && ticks < max_ticks_per_frame_) {
            const float sub_dt = fixed_dt_ / static_cast<float>(substeps_);
            for (int s = 0; s < substeps_; s++) {
                // single substep stack
                {
                    ScopeTimer substep_timer = dbg_ ? dbg_->timer_substep() : ScopeTimer(nullptr);
                    Step(sub_dt);
                }
            }
            sim_accum_ -= fixed_dt_;
            ++ticks;
        }
    }

    if (dbg_) dbg_->end_step();

    OnUpdate(ctx);

    renderHelper_.Update(scene_->state_out());
}

void Sample::Render([[maybe_unused]]AppContext &ctx) {
    BeginMode3D(ctx.orbitCam->camera);

    // floor
    if (RenderHelper::IsModelValid(floor_)) {
        DrawModel(floor_, Vector3{0,0,0}, 1.0f, WHITE);
    }

    // scene models
    renderHelper_.Draw(ctx.is_wire_mode);

    EndMode3D();
}

void Sample::DrawUI([[maybe_unused]]AppContext &ctx) {

    if (dbg_)
        dbg_ui_.Render(*dbg_);


}

void Sample::Reset([[maybe_unused]]AppContext &ctx) {

    ctx.frame_id = 0;

    if (scene_ == nullptr) return;
    if (solver_ == nullptr) return;

    scene_->state_in() = scene_->model_.MakeState();
    scene_->state_out() = scene_->model_.MakeState();

    // reset solver, mainly debugger
    if (dbg_) dbg_->reset();

    // move model to init position
    if (ctx.paused)
        renderHelper_.Update(scene_->state_out());

}

void Sample::CleanUp() {
    solver_.reset();
    scene_.reset();
    dbg_.reset();
}

void Sample::BuildRenderResources() {
    if (scene_ == nullptr)
        return;

    renderHelper_.BindModel(scene_->model_);
    renderHelper_.Update(scene_->state_out());      // need update once manually in case app is paused and pass sample update in main loop
}

void Sample:: DestroyRenderResources() {
    renderHelper_.Shutdown();
    RenderHelper::UnloadRLModelSafe(floor_);  // models that have no physical meanings is owned by sample itself
}

void Sample::CreateWorld([[maybe_unused]]AppContext& ctx) {
    // just a scene with floor
    /*MModel model;
    scene_ = std::make_unique<Scene>(std::move(model));*/
}

void Sample::Step(const float dt) {

    if (scene_ == nullptr) return;
    if (solver_ == nullptr) return;

    solver_->Step(scene_->state_in(), scene_->state_out(), dt);
    scene_->SwapStates();
};

void Sample::CreateFloor([[maybe_unused]]AppContext& ctx) {

    if (ctx.shader_manager == nullptr)
        throw std::runtime_error("No shader manager found");

    ctx.shader_manager->LoadShaderProgram("floor", "../resources/shaders/floor.vs", "../resources/shaders/floor.fs");
    const auto floor_shader = ctx.shader_manager->Get("floor")->shader;
    ShaderManager::BindMatrices(floor_shader);
    ShaderManager::SetCommonShaderParams(floor_shader);
    const Mesh floor_mesh = GenMeshPlane(500.05f, 500.0f, 1, 1);
    floor_ = LoadModelFromMesh(floor_mesh);
    floor_.materials[0].shader = floor_shader;

    // locate uniform parameters
    const int tileScale = ShaderManager::CheckSetShaderLocation(floor_shader, "tileScale");
    const int lineWidth = ShaderManager::CheckSetShaderLocation(floor_shader, "lineWidth");
    const int baseAColor = ShaderManager::CheckSetShaderLocation(floor_shader, "baseAColor");
    const int baseBColor = ShaderManager::CheckSetShaderLocation(floor_shader, "baseBColor");
    const int lineColor = ShaderManager::CheckSetShaderLocation(floor_shader, "lineColor");
    const int roughness = ShaderManager::CheckSetShaderLocation(floor_shader, "roughness");
    const int bumpStrength = ShaderManager::CheckSetShaderLocation(floor_shader, "bumpStrength");
    const int fogDensity = ShaderManager::CheckSetShaderLocation(floor_shader, "fogDensity");
    const int fogColor = ShaderManager::CheckSetShaderLocation(floor_shader, "fogColor");


    // Floor appearance
    constexpr Vector3 floorFogColor  = { 0.10f, 0.13f, 0.17f }; // 用于 fogColor（线性，偏蓝灰）
    constexpr Vector3 floorBaseACol  = { 0.08f, 0.085f, 0.09f }; // 底色A：深
    constexpr Vector3 floorBaseBCol  = { 0.13f, 0.135f, 0.14f }; // 底色B：浅（噪声混合）
    constexpr Vector3 floorLineColor   = { 0.20f, 0.205f, 0.215f}; // 网格线颜色（别太亮）

    constexpr float floorRough  = 0.55f;                // 越大越哑光，反光越弱
    constexpr float floorBumpStr     = 0.22f;                // 微起伏：增强高级感（太大像橡皮泥）
    constexpr float floorFogDensity  = 0.015f;               // 雾：让地板“无穷远”+聚光更明显
    constexpr float floorTileScale   = 2.0f;                 // 网格密度：越大格子越小
    constexpr float floorLineWidth   = 0.035f;               // 网格线宽：越大线越明显

    SetShaderValue(floor_shader, tileScale, &floorTileScale, SHADER_UNIFORM_FLOAT);
    SetShaderValue(floor_shader, lineWidth, &floorLineWidth, SHADER_UNIFORM_FLOAT);
    SetShaderValue(floor_shader, baseAColor, &floorBaseACol, SHADER_UNIFORM_VEC3);
    SetShaderValue(floor_shader, baseBColor, &floorBaseBCol, SHADER_UNIFORM_VEC3);
    SetShaderValue(floor_shader, lineColor, &floorLineColor, SHADER_UNIFORM_VEC3);

    SetShaderValue(floor_shader, roughness, &floorRough, SHADER_UNIFORM_FLOAT);
    SetShaderValue(floor_shader, bumpStrength, &floorBumpStr, SHADER_UNIFORM_FLOAT);
    SetShaderValue(floor_shader, fogDensity, &floorFogDensity, SHADER_UNIFORM_FLOAT);
    SetShaderValue(floor_shader, fogColor, &floorFogColor, SHADER_UNIFORM_VEC3);
}





