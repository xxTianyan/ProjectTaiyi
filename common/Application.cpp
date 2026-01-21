//
// Created by xumiz on 2026/1/1.
//

#include <stdexcept>
#include "Application.h"
#include "raylib.h"
#include "imgui.h"
#include "rlImGui.h"
#include "CameraController.h"


Application::Application(const Desc &desc) :
        desc_(desc),
        orbitCam_(CreateOrbitCamera(Vector3{ 6.0f, 7.0f, 0.0f }, Vector3{ 0.0f, 0.0f, 0.0f })) {

    // bind cxt
    ctx_.shader_manager = &shader_manager_;
    ctx_.orbitCam = &orbitCam_;
    ctx_.target_fps = desc_.target_fps;
}

Application::~Application() {
    if (IsWindowReady()) {
        ExitCurrentSample_();
        ShutdownWindowAndUI_();
    }
}

void Application::InitWindowAndUI_() const {
    // Window flags
    if (desc_.resizable) SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    SetConfigFlags(FLAG_MSAA_4X_HINT); // anti alias
    InitWindow(desc_.width, desc_.height, desc_.title);
    SetTargetFPS(desc_.target_fps);

    // rlImGui init
    rlImGuiSetup(true); // true: enable docking if supported by your build
}

void Application::ShutdownWindowAndUI_() {
    rlImGuiShutdown();
    CloseWindow();
}

void Application::EnterSample_(const SampleId id) {
    current_id_ = id;
    current_ = registry_.Create(id);
    if (!current_) throw std::runtime_error("Create sample returned null");

    current_->OnEnter(ctx_);

    ctx_.frame_id = 0;
}

void Application::ExitCurrentSample_() {
    if (current_) {
        current_->OnExit(ctx_);
        current_.reset();
    }
}

void Application::RequestSwitchSample_(const SampleId id) {
    if (current_id_ == id) return;
    pending_.type = PendingActionType::SwitchSample;
    pending_.target = id;
}

void Application::RequestReloadSample_() {
    pending_.type = PendingActionType::ReloadSample;
    pending_.target = current_id_;
}

void Application::PollHotkeys_() {

    /* Reset
    if (IsKeyPressed(KEY_R)) {
        RequestReloadSample_();
    }
    // Pause
    if (IsKeyPressed(KEY_SPACE)) {
        pending_.type = PendingActionType::TogglePause;
    }
    // Reload shaders
    if (IsKeyPressed(KEY_F5)) {
        pending_.type = PendingActionType::ReloadShaders;
    }

    // 快捷切 sample（示例：1/2/3）
    if (IsKeyPressed(KEY_ONE))  RequestSwitchSample_(SampleId::ClothDrop);
    if (IsKeyPressed(KEY_TWO))  RequestSwitchSample_(SampleId::TetStVK);*/
}

void Application::DrawAppUI_() {

    ImGui::Begin("Application");

    ImGui::Text("Current: %s", current_ ? current_->Name() : "(none)");
    ImGui::Separator();

    // Pause / Reset / Reload shaders
    if (ImGui::Checkbox("Paused", &ctx_.paused)) {
    }
    if (ImGui::Button("Reload Sample")) {
        RequestReloadSample_();
    }
    if (ImGui::Button("Wire Mode")) {
        ctx_.is_wire_mode = !ctx_.is_wire_mode;
    }

    ImGui::Separator();
    ImGui::Text("Switch Sample:");

    // pull down to choose sample
    static int current_index = 0;
    if (const auto& infos = registry_.Infos(); !infos.empty()) {
        // 同步 index（当你按热键切换后，UI 也要跟上）
        for (int i = 0; i < static_cast<int>(infos.size()); ++i) {
            if (infos[i].id == current_id_) { current_index = i; break; }
        }

        if (ImGui::BeginCombo("Sample", infos[current_index].display_name)) {
            for (int i = 0; i < static_cast<int>(infos.size()); ++i) {
                const bool selected = (i == current_index);
                if (ImGui::Selectable(infos[i].display_name, selected)) {
                    current_index = i;
                    RequestSwitchSample_(infos[i].id);
                }
                if (selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    } else {
        ImGui::Text("No samples registered.");
    }



    ImGui::End();

    // if Sample own UI
    if (current_) {
        current_->DrawUI(ctx_);
    }

    // draw performance monitor
    perfMonitor_.Draw();
}

void Application::ExecutePending_() {
    switch (pending_.type) {
        case PendingActionType::None:
            return;

        case PendingActionType::TogglePause:
            ctx_.paused = !ctx_.paused;
            break;

        case PendingActionType::ReloadShaders:
            // 注意：如果 Sample 内部缓存了 shader location/uniform handle，
            // 你需要让 Sample 在下一帧重新抓取，或把该逻辑放到 ShaderManager 封装里。
            // shader_manager_.ReloadAll();
            break;

        case PendingActionType::ReloadSample: {
           current_->Reset(ctx_);
        } break;

        case PendingActionType::SwitchSample: {
            ExitCurrentSample_();
            EnterSample_(pending_.target);
        } break;
    }

    pending_.type = PendingActionType::None;
}

void Application::Run(const SampleId start_sample) {
    InitWindowAndUI_();

    // init sample
    EnterSample_(start_sample);

    while (!WindowShouldClose()) {

        const float dt = GetFrameTime();
        ctx_.dt = dt;

        // common updates
        perfMonitor_.Update(dt);
        RefreshCameraTransform(orbitCam_);

        // key event
        PollHotkeys_();
        if (!ImGui::GetIO().WantCaptureKeyboard) {
            UpdateOrbitCameraKeyboard(orbitCam_, dt, 10.0f);
        }

        // mouse event
        if (!ImGui::GetIO().WantCaptureMouse) {
            auto [x, y] = GetMouseDelta();
            const float wheel = GetMouseWheelMove();
            UpdateOrbitCameraMouse(orbitCam_, Vector2{x, y}, wheel,
                                   IsMouseButtonDown(MOUSE_RIGHT_BUTTON), IsMouseButtonDown(MOUSE_MIDDLE_BUTTON));
        }

        // Update
        if (current_ && !ctx_.paused) {
            current_->Update(ctx_);
            ctx_.frame_id++;
        }

        // Render
        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawRectangleGradientV(
            0, 0, GetScreenWidth(), GetScreenHeight(),
            Color{ 45, 93, 125, 255 },   // top
            Color{ 10, 12, 16, 255 }    // bottom
            );

        if (current_) {
            current_->Render(ctx_);
        }

        // ImGui
        rlImGuiBegin();
        DrawAppUI_();
        rlImGuiEnd();

        EndDrawing();

        // delay executing
        ExecutePending_();

    }

    // careful with exiting
    shader_manager_.UnloadAll();
    ExitCurrentSample_();
    ShutdownWindowAndUI_();
}
