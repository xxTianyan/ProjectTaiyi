//
// Created by tianyan on 1/15/26.
//

#ifndef TAIYI_DEBUGGER_HPP
#define TAIYI_DEBUGGER_HPP

#include <limits>
#include <fstream>
#include <vector>
#include "Types.h"
#include <chrono>

struct TimerStat {
    double sum_ms = 0.0;
    uint32_t count = 0;

    double avg_ms() const { return count ? (sum_ms / static_cast<double>(count)) : 0.0; }
};


struct ScopeTimer {
    using clock = std::chrono::steady_clock;

    TimerStat* stat = nullptr;
    clock::time_point t0;

    explicit ScopeTimer(TimerStat* s) noexcept : stat(s), t0(clock::now()) {}

    ~ScopeTimer() noexcept {
        if (!stat) return;
        const auto t1 = clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        stat->sum_ms += ms;
        stat->count  += 1;
    }

    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;
};

// --- 配置结构 ---
struct DebugTriggerConfig {
    bool break_on_nan = true;         // 遇到 NaN 立刻暂停
    bool break_on_inversion = true;   // 遇到翻转 (Signed Vol < 0) 立刻暂停
    bool break_on_small_J = false;      // 遇到 J 小于 0.1 触发
    float dx_limit_scale = 0.5f;      // 允许稍大的位移，过于灵敏会频繁打断
    float max_pen_trigger = .8f;     // 穿透深度阈值
};

// --- 错误类型 ---
enum class DebugErrorType : uint8_t {
    None,
    NaN_Detected,
    Inverted_Element,
    Large_Deformation,
    Large_Penetration,
    Low_Jacobian,
    User_Trigger
};

// --- 单帧统计数据 ---
struct DebugFrameStats {
    size_t frame_id = 0;

    // 统计极值 (用于可视化热力图范围等)
    float minJ = std::numeric_limits<float>::infinity();
    float minSignedVol = std::numeric_limits<float>::infinity();
    float maxPenetration = -std::numeric_limits<float>::infinity();
    float maxDx = 0.0f;

    // --- 性能统计 (毫秒) ---
    TimerStat time_total_physical_frame;     // 整帧耗时
    TimerStat time_single_substep;   // 所有子步总耗时
    TimerStat time_single_iteration;
    TimerStat time_linear_solve;    // 线性方程组求解耗时
    TimerStat time_gradient_update; // 梯度/Hessian构建耗时

    // 触发信息 (记录到底是谁导致了暂停)
    DebugErrorType trigger_reason = DebugErrorType::None;
    size_t trigger_element_id = -1; // Vertex ID or Tet ID
    std::string trigger_msg;        // 详细描述

    // 现场数据 snapshot (导致错误的那个点的物理量)
    std::vector<std::pair<std::string, Vec3>> component_forces;
    std::vector<std::pair<std::string, Mat3>> component_hessians;
    Vec3 trigger_force{};
    Mat3 trigger_hessian{};
    float trigger_val_1 = 0.0f;     // 通用槽位，存 J, vol 或 dx

    void reset(const size_t fid) {
        *this = DebugFrameStats{}; //通过重置整个结构体清零时间
        frame_id = fid;
        minJ = std::numeric_limits<float>::infinity();
        minSignedVol = std::numeric_limits<float>::infinity();
        maxPenetration = -std::numeric_limits<float>::infinity();
    }
};

class SolverDebugger {
public:
    enum RunState : uint8_t { Running, Frozen };

    explicit SolverDebugger(const size_t history_capacity = 200) {
        frame_history_.resize(history_capacity);
    }
    ~SolverDebugger() = default;

    // --- 流程控制 ---

    // 在 Solver Step 开始前调用。返回 false 表示不应该执行物理 Step
    bool begin_step(const size_t frame_id) {
        frame_id_ = frame_id;

        // 如果之前被 UI 设为“单步模式”，这一帧允许通过，但下一帧要锁住
        if (step_once_armed_) {
            state_ = RunState::Running; // 临时放行
        }

        if (state_ == RunState::Frozen) {
            return false; // Solver 应该跳过 Update
        }

        // 准备新的一帧数据
        current_frame_.reset(frame_id);
        triggered_this_frame_.store(false);
        return true;
    }

    // 在 Solver Step 结束后调用
    void end_step() {
        if (state_ == RunState::Frozen) return; // 没跑，不用记录

        // 如果在计算过程中触发了 Trigger
        if (triggered_this_frame_.load()) {
            state_ = RunState::Frozen; // 锁死
            step_once_armed_ = false;  // 取消单步
        }

        else if (step_once_armed_) {
            // 如果是单步执行且没报错，执行完这一次后，切回 Frozen
            state_ = RunState::Frozen;
            step_once_armed_ = false;
        }

        // 写入历史 (Ring Buffer)
        const size_t ptr = frame_id_ % frame_history_.size();
        frame_history_[ptr] = current_frame_;
    }

    // UI 按钮: "Step One Frame"
    void ui_step_one() {
        step_once_armed_ = true;
    }

    // UI 按钮: "Continue / Resume"
    void ui_continue() {
        state_ = RunState::Running;
        step_once_armed_ = false;
    }

    // UI 按钮: "Pause"
    void ui_pause() {
        state_ = RunState::Frozen;
    }

    bool is_frozen() const { return state_ == RunState::Frozen; }
    const DebugFrameStats& get_current_stats() const { return current_frame_; }

    // ---- Timer -----
    ScopeTimer timer_frame() { return ScopeTimer(&current_frame_.time_total_physical_frame); }
    ScopeTimer timer_substep() { return ScopeTimer(&current_frame_.time_single_substep); }
    ScopeTimer timer_iteration() { return ScopeTimer(&current_frame_.time_single_iteration); }
    ScopeTimer timer_linear_solve() { return ScopeTimer(&current_frame_.time_linear_solve); }
    ScopeTimer timer_gradient() { return ScopeTimer(&current_frame_.time_gradient_update); }

    // 通用接口，如果你想计时一些临时变量
    ScopeTimer timer_custom(TimerStat* target_ptr) { return ScopeTimer(target_ptr); }

    // --- Solver 内部检测函数 (支持多线程调用) ---

    // 检查顶点数据
    void inspect_vertex(const size_t v_id, const Vec3& force, const Mat3& hessian, const float dx, const float penetration, const float avg_len) {
        if (triggered_this_frame_.load()) return;

        // 1. 快速无锁检查 (Performance Critical)
        const bool has_nan = !std::isfinite(dx) || !std::isfinite(penetration) ||
                       !std::isfinite(force.x()) || !std::isfinite(force.y()) || !std::isfinite(force.z());

        if (penetration > current_frame_.maxPenetration) current_frame_.maxPenetration = penetration;
        if (dx > current_frame_.maxDx) current_frame_.maxDx = dx;

        current_frame_.trigger_force = force;
        current_frame_.trigger_hessian = hessian;

        // 2. 触发逻辑
        if (has_nan) {
            report_trigger(DebugErrorType::NaN_Detected, v_id, "NaN in Vertex (dx/pen/force)", dx);

            return;
        }

        if (dx > cfg_.dx_limit_scale * avg_len) {
            report_trigger(DebugErrorType::Large_Deformation, v_id, "Large dx detected", dx);
        }
    }

    // 检查四面体数据
    bool inspect_tet(const size_t t_id, const float J, const float signed_vol) {
        const bool has_nan = !std::isfinite(J) || !std::isfinite(signed_vol);

        if (J < current_frame_.minJ) current_frame_.minJ = J;
        if (signed_vol < current_frame_.minSignedVol) current_frame_.minSignedVol = signed_vol;

        if (has_nan) {
            report_trigger(DebugErrorType::NaN_Detected, t_id, "NaN in Tet (J/Vol)", J);
            return true;
        }

        if (cfg_.break_on_inversion && signed_vol <= 0.0f) {
            report_trigger(DebugErrorType::Inverted_Element, t_id, "Element Inverted (Vol < 0)", signed_vol);
            return true;
        }

        if (cfg_.break_on_small_J && J < 1e-5) {
            report_trigger(DebugErrorType::Low_Jacobian, t_id, "Low Jacobian", J);
            return true;
        }

        return false;
    }

    // --- Dump 功能 ---

    // 导出历史记录到 JSON 格式
    void dump_history_json(const std::string& filepath) const {
        std::ofstream out(filepath);
        out << "{\n  \"frames\": [\n";

        // 遍历 Ring Buffer (按时间顺序)
        // 注意：这里只存了 stats，如果需要回放画面，你需要在这里把 Solver 的 x 数组也存下来
        // 这通常需要传入 Solver 的引用或回调函数。

        const size_t start = (frame_id_ >= frame_history_.size()) ? (frame_id_ + 1) % frame_history_.size() : 0;
        const size_t count = std::min(frame_id_ + 1, frame_history_.size());

        for (size_t i = 0; i < count; ++i) {
            const size_t idx = (start + i) % frame_history_.size();
            const auto& frame = frame_history_[idx];

            out << "    { \"id\": " << frame.frame_id
                << ", \"minJ\": " << frame.minJ
                << ", \"trigger\": \"" << static_cast<int>(frame.trigger_reason) << "\" "
                << "}" << (i < count - 1 ? "," : "") << "\n";
        }
        out << "  ]\n}\n";
    }

    // --- 重置功能 ---
    void reset() {
        // 1. 线程安全锁：防止 UI 重置时，后台恰好还在写入最后一帧数据
        std::lock_guard<std::mutex> lock(data_mutex_);

        // 2. 恢复运行状态
        state_ = RunState::Running;      // 恢复为 Running
        step_once_armed_ = false;        // 清除单步标志
        triggered_this_frame_.store(false); // 清除原子 Trigger 标志

        // 3. 重置计数器
        frame_id_ = 0;

        // 4. 重置当前帧数据
        current_frame_.reset(0);

        // 5. 清空历史记录 (Ring Buffer)
        // 我们不 resize vector (避免内存分配开销)，而是将内容恢复为初始默认值
        // 这样 UI 在读取历史时，不会读到上一次运行遗留的“幽灵数据”
        for (auto& frame : frame_history_) {
            // 这里使用 DebugFrameStats 的默认构造函数覆盖旧数据
            // 默认构造会将 minJ 设为 infinity，maxPen 设为负无穷等安全值
            frame = DebugFrameStats{};
        }

        printf("[Debugger] System Reset. State: RUNNING, Frame: 0.\n");
    }

public:
    DebugTriggerConfig cfg_;

private:
    // --- 内部辅助 ---

    // 线程安全的 Trigger 报告
    void report_trigger(const DebugErrorType type, const size_t id, const std::string& msg, const float val) {
        // Double-Check Locking: 只有第一个触发的人能写入，避免被后续报错覆盖
        bool expected = false;
        if (triggered_this_frame_.compare_exchange_strong(expected, true)) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            current_frame_.trigger_reason = type;
            current_frame_.trigger_element_id = id;
            current_frame_.trigger_msg = msg;
            current_frame_.trigger_val_1 = val;

            // 可选：在这里 print 到控制台，因为 Solver 可能马上就崩溃了
            printf("[Debugger] FROZEN at frame %zu. Reason: %s (ID: %zu, Val: %f)\n",
                   frame_id_, msg.c_str(), id, val);
        }
    }

    void record_force_hessian(const Vec3& inertia_f, const Vec3& stvk_tri_f, const Vec3& NH_tet_f, const Vec3& contact_f, const Vec3 total_f,
        const Mat3& inertia_H, const Mat3& stvk_tri_H, const Mat3& NH_tet_H, const Mat3& contact_H, const Mat3& total_H) {
        current_frame_.component_forces.emplace_back("Inertia Force", inertia_f);
        current_frame_.component_forces.emplace_back("STVK Tri Force", stvk_tri_f);
        current_frame_.component_forces.emplace_back("Neo-Hookean Force", NH_tet_f);
        current_frame_.component_forces.emplace_back("Contact Force", contact_f);
        current_frame_.component_hessians.emplace_back("Inertia Hessian", inertia_H);
        current_frame_.component_hessians.emplace_back("STVK Tri Hessian", stvk_tri_H);
        current_frame_.component_hessians.emplace_back("Neo-Hookean Tet Hessian", NH_tet_H);
        current_frame_.component_hessians.emplace_back("Contact Hessian", contact_H);
        current_frame_.trigger_force = total_f;
        current_frame_.trigger_hessian = total_H;
    }

    RunState state_ = RunState::Running;
    bool step_once_armed_ = false;
    size_t frame_id_ = 0;

    std::atomic<bool> triggered_this_frame_{false}; // 原子标志位，用于快速检查
    std::mutex data_mutex_;                         // 写 Trigger 详情时加锁

    DebugFrameStats current_frame_;
    std::vector<DebugFrameStats> frame_history_;
};



#endif //TAIYI_DEBUGGER_HPP