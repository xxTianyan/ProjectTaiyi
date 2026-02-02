//
// Created by xumiz on 2025/12/24.
//

#include "Sample.h"
#include "Application.h"
#include "VBDSolver.h"

static void DrawStatRow(const char* label, size_t count, const char* tooltip = nullptr) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "%s", label); // 灰色标签
    if (tooltip && ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tooltip);

    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%zu", count); // 显示数值
}

static void DrawStatRow(const char* label, float value, const char* format = "%.4f") {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "%s", label);
    ImGui::TableSetColumnIndex(1);
    ImGui::Text(format, value);
}

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
    if (scene_) {
        renderHelper_.Draw(scene_->state_out(), ctx.is_wire_mode);
    }

    EndMode3D();
}

void Sample::DrawUI([[maybe_unused]]AppContext &ctx) {

    if (scene_ == nullptr) return;

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    static int ui_fps = static_cast<int>(1.0f / fixed_dt_) + 1;

    // 1. 【位置便利贴】
    // 意思：把窗口的右上角(Pivot=1,0)，钉在屏幕的工作区右上角
    // 这样当你向左拉伸宽度时，右边是固定的，窗口会向左长。
    ImGui::SetNextWindowPos(
        ImVec2(viewport->WorkPos.x + viewport->WorkSize.x, viewport->WorkPos.y),
        ImGuiCond_Always,
        ImVec2(1.0f, 0.0f) // Pivot: 1.0f=右边缘, 0.0f=上边缘
    );

    // 2. 【尺寸约束便利贴】 (这是解决你高度问题的关键!)
    // 意思：宽度的最小值是 200，最大值无限(FLT_MAX) -> 用户可以自由拉宽
    //      高度的最小值是 viewport.y，最大值也是 viewport.y -> 高度被死死锁在屏幕高度上
    ImGui::SetNextWindowSizeConstraints(
        ImVec2(200.0f, viewport->WorkSize.y), // 最小尺寸 (min_w, min_h)
        ImVec2(FLT_MAX, viewport->WorkSize.y) // 最大尺寸 (max_w, max_h)
    );

    // 3. 【初始尺寸便利贴】
    // 意思：如果是第一次运行，宽度设为 320。
    // 高度写 0 或者 -1 都没关系，因为上面的 Constraints 会强制把它拉长到 viewport.y
    ImGui::SetNextWindowSize(ImVec2(640.0f, 0.0f), ImGuiCond_FirstUseEver);

    // (D) 窗口 Flag
    // NoMove: 禁止拖动标题栏移动窗口（因为我们强制固定在右侧了）
    // NoResize: 去掉这个 Flag！我们要允许用户拉伸边缘
    // NoCollapse: 禁止折叠成条
    constexpr ImGuiWindowFlags windowFlags = ImGuiWindowFlags_NoMove |
                                             ImGuiWindowFlags_NoCollapse |
                                             ImGuiWindowFlags_NoSavedSettings;

    if (ImGui::Begin("Simulation Stats", nullptr, windowFlags)) {

        // --- 2. 模拟控制与重置 ---
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "SIMULATION CONTROL");
        ImGui::Separator();

        if (ImGui::CollapsingHeader("Time Integration", ImGuiTreeNodeFlags_DefaultOpen)) {
            // 参数调整
            ImGui::Text("Physical FPS");
            ImGui::SameLine(270);
            ImGui::SetNextItemWidth(200.0f);
            if (ImGui::DragInt("##dt", &ui_fps, 1, 10, 240, "%d")) {
                if (ui_fps < 10)  ui_fps = 10;
                if (ui_fps > 240) ui_fps = 240;
                fixed_dt_ = 1.0f / static_cast<float>(ui_fps);
            }
            ImGui::SameLine();
            ImGui::Text("%.5f s", fixed_dt_);

            ImGui::Text("Substeps"); ImGui::SameLine(270);
            ImGui::SetNextItemWidth(200.0f);
            ImGui::SliderInt("##substeps", &substeps_, 1, 20);

            ImGui::Text("Max Ticks"); ImGui::SameLine(270);
            ImGui::SetNextItemWidth(200.0f);
            ImGui::SliderInt("##ticks", &max_ticks_per_frame_, 1, 10);

            ImGui::Spacing();

            // [新增] Reset Setting 按钮
            // 使用红色调稍微警示一下，或者是普通的按钮
            if (ImGui::Button("Reset Settings", ImVec2(-1, 0))) {
                fixed_dt_ = fixed_dt0_;
                ui_fps = static_cast<int>(1.0f / fixed_dt_) + 1;
                substeps_ = substeps0_;
                max_ticks_per_frame_ = max_ticks_per_frame0_;
            }

            ImGui::Spacing();
            if (ImGui::BeginTable("TimeStats", 2)) {
                ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, 270.0f);
                ImGui::TableSetupColumn("Value");

                DrawStatRow("Sim Accumulation", sim_accum_, "%.3f s");
                DrawStatRow("Frame ID", ctx.frame_id); // 假设 ctx.frame_id 是 size_t 或 int

                ImGui::EndTable();
            }

        }

        ImGui::Spacing();

        // --- 环境与统计 ---
        if (ImGui::CollapsingHeader("Environment", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::Text("Gravity");
            ImGui::SetNextItemWidth(-1);
            ImGui::DragFloat3("##Gravity", scene_->model_.gravity_.data(), 0.1f, -100.0f, 100.0f, "%.2f");
        }

        if (ImGui::CollapsingHeader("Scene Statistics"), ImGuiTreeNodeFlags_DefaultOpen) {
            const auto& model = scene_->model_;

            // 使用 Table 使数据对齐，看起来更专业
            if (ImGui::BeginTable("SceneStatsTable", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerH)) {
                // 设置列宽，第一列固定宽度
                ImGui::TableSetupColumn("Property", ImGuiTableColumnFlags_WidthFixed, 350.0f);
                ImGui::TableSetupColumn("Count", ImGuiTableColumnFlags_WidthStretch);

                // 刚体部分
                DrawStatRow("Rigid Bodies", model.num_bodies);

                // 粒子部分 (加一个 Tooltip 说明)
                DrawStatRow("Total Particles", model.num_particles, "Includes deformable and rigid particles");

                // 拓扑结构
                DrawStatRow("Tetrahedra", model.tets.size());
                DrawStatRow("Triangles (Sim)", model.tris.size());
                DrawStatRow("Triangles (Render)", model.render_tris.size());
                DrawStatRow("Edges", model.edges.size());

                ImGui::EndTable();
            }
        }

        ImGui::Spacing();
        ImGui::Separator();

        // --- 3. Debugger 整合  ---

        ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "DEBUG TOOLS");

        if (ImGui::CollapsingHeader("Solver Debugger"), ImGuiTreeNodeFlags_DefaultOpen) {
            if (dbg_) {
                ImGui::PushID("EmbeddedDbg"); // 防止ID冲突
                // 调用我们新提取的 DrawContent 函数
                dbg_ui_.DrawContent(*dbg_);
                ImGui::PopID();
            } else {
                ImGui::TextDisabled("Debugger not initialized");
            }
        }

        ImGui::End();
    }

}

void Sample::Reset([[maybe_unused]]AppContext &ctx) {

    ctx.frame_id = 0;
    sim_accum_ = 0.0f;

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

    const Contacts* contacts;
    {
        // collide here
        contacts = scene_->model_.collide(scene_->state_out());
    }

    solver_->Step(scene_->state_in(), scene_->state_out(), contacts, dt);
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





