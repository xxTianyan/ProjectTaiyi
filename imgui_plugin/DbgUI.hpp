//
// Created by tianyan on 1/16/26.
//

#ifndef TAIYI_DBGUI_HPP
#define TAIYI_DBGUI_HPP


#include <string>
#include <ctime>
#include <sstream>
#include <iomanip>
#include "imgui.h"
#include "Debugger.hpp"

class SolverDebuggerUI {
public:
    // --- 主渲染函数 ---
    // 在你的 ImGui 循环中调用此函数
    // p_open: 可选的关闭窗口布尔指针
    void Render(SolverDebugger& debugger, bool* p_open = nullptr) {
        if (!ImGui::Begin("Solver Debugger Panel", p_open)) {
            ImGui::End();
            return;
        }

        DrawStatusAndControls(debugger);
        DrawManualDump(debugger);
        DrawTriggerInfo(debugger);
        DrawLiveStats(debugger);
        DrawPerformance(debugger);
        DrawSettings(debugger);

        ImGui::End();
    }

    void DrawContent(SolverDebugger& debugger) {
        // when want dbg panel insert in other imgui panel
        DrawStatusAndControls(debugger);
        DrawManualDump(debugger);
        DrawTriggerInfo(debugger);
        DrawLiveStats(debugger);
        DrawPerformance(debugger);
        DrawSettings(debugger);
    }

private:
    // --- 内部 UI 组件 ---

    void DrawStatusAndControls(SolverDebugger& debugger) {
        const auto& stats = debugger.get_current_stats();
        bool frozen = debugger.is_frozen();

        // 1. 状态灯
        ImGui::Text("Sim State: ");
        ImGui::SameLine();
        if (frozen) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "[ FROZEN ]");
            if (stats.trigger_reason != DebugErrorType::None) {
                ImGui::SameLine();
                ImGui::TextDisabled("(Triggered by Error)");
            }
        } else {
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "[ RUNNING ]");
        }

        ImGui::Separator();

        // 2. VCR 控制器
        // Resume
        ImGui::BeginDisabled(!frozen);
        if (ImGui::Button("Resume (F5)")) {
            debugger.ui_continue();
        }
        ImGui::EndDisabled();

        ImGui::SameLine();

        // Pause
        ImGui::BeginDisabled(frozen);
        if (ImGui::Button(" Frozen ")) {
            debugger.ui_pause();
        }
        ImGui::EndDisabled();

        ImGui::SameLine();

        // Step
        if (ImGui::Button(" Step One ")) {
            debugger.ui_step_one();
        }
    }

    void DrawManualDump(SolverDebugger& debugger) {
        ImGui::Separator();
        ImGui::AlignTextToFramePadding();
        ImGui::Text("Snapshots:");
        ImGui::SameLine();

        if (ImGui::Button("Dump JSON History")) {
            std::string fname = GenerateTimestampFilename("debug_dump", ".json");
            debugger.dump_history_json(fname);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Save current history ring buffer to JSON");
        }
    }

    void DrawTriggerInfo(SolverDebugger& debugger) {
        const auto& stats = debugger.get_current_stats();
        bool frozen = debugger.is_frozen();

        // 只有在冻结且有错误时才显示
        if (!frozen || stats.trigger_reason == DebugErrorType::None) return;

        ImGui::Spacing();
        ImGui::Separator();

        // --- 1. 醒目的红色标题栏 ---
        ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.85f, 0.2f, 0.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
        bool show_details = ImGui::CollapsingHeader("!!! SIMULATION CRASH REPORT !!!", ImGuiTreeNodeFlags_DefaultOpen);
        ImGui::PopStyleColor(2);

        if (!show_details) return;

        ImGui::Indent(); // 整体缩进，增加层次感

        // --- 2. 基础信息表格 ---
        if (ImGui::BeginTable("CrashBasic", 2, ImGuiTableFlags_BordersInnerH)) {
            ImGui::TableSetupColumn("Label", ImGuiTableColumnFlags_WidthFixed, 200.0f);
            ImGui::TableSetupColumn("Value");

            // Helper Lambda
            auto DrawRow = [](const char* label, const char* val, const ImVec4& color) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextDisabled("%s", label);
                ImGui::TableSetColumnIndex(1); ImGui::TextColored(color, "%s", val);
            };

            DrawRow("Reason", stats.trigger_msg.c_str(), ImVec4(1, 0.4f, 0.4f, 1)); // Red

            if (stats.trigger_element_id != static_cast<size_t>(-1)) {
                std::string id_str = std::to_string(stats.trigger_element_id);
                DrawRow("Element ID", id_str.c_str(), ImVec4(1, 1, 0, 1)); // Yellow
            }

            // 如果有特定的触发值 (比如 Jacobian 值)
            if (std::abs(stats.trigger_val_1) > 1e-9) {
                char val_buf[32]; snprintf(val_buf, 32, "%.6f", stats.trigger_val_1);
                DrawRow("Bad Value", val_buf, ImVec4(0.7f, 0.7f, 0.7f, 1));
            }

            ImGui::EndTable();
        }

        ImGui::Spacing();
        ImGui::Separator();

        // --- 3. Force Breakdown (力的分析) ---
        ImGui::TextColored(ImVec4(0.6f, 1.0f, 0.6f, 1.0f), "FORCE COMPOSITION");

        // 3列：Name | Vector | Magnitude
        if (ImGui::BeginTable("ForceTable", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable)) {
            ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 50.0f);
            ImGui::TableSetupColumn("Vector (x, y, z)");
            ImGui::TableSetupColumn("Norm", ImGuiTableColumnFlags_WidthFixed, 60.0f);
            ImGui::TableHeadersRow();

            auto DrawForceRow = [](const char* name, const Vec3& f, bool highlight) {
                ImGui::TableNextRow();

                // Col 0: Name
                ImGui::TableSetColumnIndex(0);
                if (highlight) ImGui::TextColored(ImVec4(1, 0.8f, 0.2f, 1), "%s", name); // Gold
                else           ImGui::Text("%s", name);

                // Col 1: Vector
                ImGui::TableSetColumnIndex(1);
                ImGui::Text("(% .2f, % .2f, % .2f)", f.x(), f.y(), f.z());

                // Col 2: Norm
                ImGui::TableSetColumnIndex(2);
                ImGui::Text("%.2f", f.norm());
            };

            // A. 先画 Total Force (高亮)
            DrawForceRow("TOTAL NET", stats.trigger_force, true);

            // B. 遍历 component_forces 画分量
            for (const auto& [name, vec] : stats.component_forces) {
                DrawForceRow(name.c_str(), vec, false);
            }

            ImGui::EndTable();
        }

        ImGui::Spacing();
        ImGui::Separator();

        // --- 4. Hessian Analysis (刚度矩阵分析) ---
        ImGui::TextColored(ImVec4(0.4f, 0.8f, 1.0f, 1.0f), "HESSIAN CONTRIBUTION");
        ImGui::TextDisabled("(Red on diagonal = Indefinite/Negative Definite)");

        // A. 先画 Total Hessian (如果有数据)
        // 假设 stats.total_hessian 是你手动累加或者从 vector 里算出来的
        // 如果你没有专门存 total，可以自己在这里累加一下用于显示
        Mat3 sum_hessian = Mat3::Zero();
        // 如果 stats 里有 total_hessian 字段最好，没有的话我们现场算：
        for(const auto& pair : stats.component_hessians) sum_hessian += pair.second;

        DrawHessianGrid("TOTAL HESSIAN (Sum)", sum_hessian, true);

        // B. 遍历 component_hessians 画详细数据
        for (const auto& [name, mat] : stats.component_hessians) {
            DrawHessianGrid(name.c_str(), mat, false);
        }

        ImGui::Unindent(); // 结束缩进
    }

    void DrawLiveStats(const SolverDebugger& debugger) {
        if (!ImGui::CollapsingHeader("Live Statistics", ImGuiTreeNodeFlags_DefaultOpen)) return;

        const auto& stats = debugger.get_current_stats();

        ImGui::Text("Frame: %zu", stats.frame_id);

        auto StatRow = []<typename T0>(const char* name, T0 val, auto limit, bool is_min_limit) {
            // 1. 逻辑判断：通用比较，适用于 int 和 float
            bool bad = is_min_limit ? (val < limit) : (val > limit);

            ImGui::Text("%s:", name);
            ImGui::SameLine(300);

            // 2. 根据 bad 状态设置颜色
            if (bad) ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.3f, 0.3f, 1.0f));

            // 3. 编译期类型分流：根据 val 类型选择打印格式
            // 使用 if constexpr 保证编译器不会抱怨类型不匹配
            if constexpr (std::is_floating_point_v<T0>) {
                ImGui::Text("%.4f%s", val, bad ? " (BAD)" : "");
            } else {
                ImGui::Text("%d%s", val, bad ? " (BAD)" : "");
            }

            if (bad) ImGui::PopStyleColor();
        };

        StatRow("Min Jacobian", stats.minJ, 0.1f, true);
        StatRow("Min Volume", stats.minSignedVol, 0.0f, true);
        StatRow("Max Penetration", stats.maxPenetration, 1e-3f, false);
        StatRow("Max Dx", stats.maxDx, 1.0f, false);
        StatRow("Collision Particles", stats.collision_particles, 0, true);
    }

    void DrawSettings(SolverDebugger& debugger) {
        if (!ImGui::CollapsingHeader("Debugger Config", ImGuiTreeNodeFlags_DefaultOpen)) return;

        DebugTriggerConfig& cfg = debugger.cfg_;

        ImGui::TextDisabled("Triggers (Auto-Freeze)");
        ImGui::Checkbox("NaN Detected", &cfg.break_on_nan);
        ImGui::Checkbox("Inverted", &cfg.break_on_inversion);
        ImGui::Checkbox("Extreme J", &cfg.break_on_small_J);

        ImGui::Separator();
        ImGui::TextDisabled("Thresholds");

        // 使用 SliderFloat 而不是 DragFloat，可以防止用户拖出离谱的值
        ImGui::SliderFloat("Max Penetration", &cfg.max_pen_trigger, 0.0f, 1.0f);
        ImGui::SliderFloat("Dx Limit Scale", &cfg.dx_limit_scale, 0.01f, 2.0f);
    }

    void DrawPerformance(const SolverDebugger& debugger) {
        if (!ImGui::CollapsingHeader("Performance (ms)", ImGuiTreeNodeFlags_DefaultOpen)) return;

        const auto& stats = debugger.get_current_stats();

        // Lambda: 增加了一个 color 参数
        auto TimeBar = [](const char* label, double ms, double total_ms, const ImVec4& color) {
            ImGui::Text("%s:", label);
            ImGui::SameLine(300); // 对齐数值
            ImGui::Text("%.3f ms", ms);

            if (total_ms > 0.0001) {
                float fraction = static_cast<float>(ms / total_ms);
                fraction = std::clamp(fraction, 0.0f, 1.0f); // 防止溢出

                ImGui::SameLine();

                // --- 核心修改：改变颜色 ---
                // ImGuiCol_PlotHistogram 控制进度条填充部分的颜色
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);

                // 绘制进度条 (高度设为 ImGui::GetTextLineHeight() 让它看起来不像默认那么厚，更精致)
                ImGui::ProgressBar(fraction, ImVec2(-1, ImGui::GetTextLineHeight() * 0.8f), "");

                ImGui::PopStyleColor(); // 记得立刻还原，否则会影响后续控件
                // -----------------------
            }
        };

        const ImVec4 col_ = ImVec4(0.7f, 0.7f, 0.7f, 1.0f); // 灰色

        // 显示总时间
        ImGui::Text("Physical Frame:   %.3f ms", stats.time_total_physical_frame.avg_ms());
        ImGui::Separator();

        // 显示各部分时间 (以 Total Frame 为分母计算条形图)
        TimeBar("One Substep", stats.time_single_substep.avg_ms(), stats.time_total_physical_frame.avg_ms(), col_);

        // ImGui::Indent();
        TimeBar("One Iteration", stats.time_single_iteration.avg_ms(), stats.time_total_physical_frame.avg_ms(), col_);
        TimeBar("Accumulate Gradient", stats.time_gradient_update.avg_ms(), stats.time_total_physical_frame.avg_ms(), col_);
        TimeBar("Linear Solve", stats.time_linear_solve.avg_ms(), stats.time_total_physical_frame.avg_ms(), col_);
        // TimeBar("Custom Timer", stats.time_random.avg_ms(), stats.time_total_physical_frame.avg_ms(), col_);
        // ImGui::Unindent();
    }

    // --- 工具函数 ---
    std::string GenerateTimestampFilename(const std::string& prefix, const std::string& ext) {
        auto now = std::time(nullptr);
        auto tm = *std::localtime(&now);
        std::ostringstream oss;
        oss << prefix << "_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ext;
        return oss.str();
    }

    // =========================================================
    //  Helper: 画一个 3x3 矩阵 (带对角线负值警示)
    // =========================================================
    static void DrawHessianGrid(const char* label, const Mat3& H, bool is_total = false) {
        // 如果是总矩阵，默认展开；如果是分量，默认折叠
        ImGuiTreeNodeFlags flags = is_total ? ImGuiTreeNodeFlags_DefaultOpen : ImGuiTreeNodeFlags_None;

        // 计算 Frobenius 范数，展示在标题上，方便一眼看出哪个矩阵数值炸了
        float frob_norm = 0.0f;
        for(int i=0; i<9; ++i) frob_norm += H.data()[i] * H.data()[i];
        frob_norm = std::sqrt(frob_norm);

        // 标题格式： "Name [Norm: 1.23e+05]"
        std::string header_label = std::string(label) + " [Norm: " + std::to_string((int)frob_norm) + "]";

        if (ImGui::TreeNodeEx(header_label.c_str(), flags)) {

            // 使用 Table 保证 3x3 对齐
            // SizingFixedFit 让列宽紧凑
            if (ImGui::BeginTable("HGrid", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {

                for (int r = 0; r < 3; ++r) {
                    ImGui::TableNextRow();
                    for (int c = 0; c < 3; ++c) {
                        ImGui::TableSetColumnIndex(c);
                        float val = H(r, c);

                        // --- 颜色逻辑 ---
                        ImVec4 color;
                        if (r == c && val < 0) {
                            // 严重警告：对角线为负 (非正定风险)
                            color = ImVec4(1.0f, 0.3f, 0.3f, 1.0f);
                        } else if (std::abs(val) < 1e-6f) {
                            // 几乎为 0，变暗
                            color = ImVec4(0.5f, 0.5f, 0.5f, 0.5f);
                        } else {
                            // 正常数值，蓝色调
                            color = ImVec4(0.4f, 0.8f, 1.0f, 1.0f);
                        }

                        ImGui::TextColored(color, "% .2e", val);

                        // 鼠标悬停显示精确浮点值
                        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%.6f", val);
                    }
                }
                ImGui::EndTable();
            }
            ImGui::TreePop();
        }
    }
};


#endif //TAIYI_DBGUI_HPP