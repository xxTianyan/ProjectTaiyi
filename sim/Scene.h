//
// Created by 徐天焱 on 2025/11/11.
//

#ifndef TAIYI_SCENE_H
#define TAIYI_SCENE_H

// all static values are stored in model struct, all changing values are stored in state struct
#include <array>
#include <utility>  // std::swap
#include "Model.h"
#include "Types.h"

class Scene {
public:
    explicit Scene(MModel&& model)
        : model_(std::move(model)) {
        // double cache
        state_buf_[0] = model_.MakeState();
        state_buf_[1] = model_.MakeState();
        SetGravity(Vec3{0.0f, -9.81f, 0.0f});
    }

    [[nodiscard]] State& state_in() noexcept { return state_buf_[in_idx_]; }
    [[nodiscard]] State& state_out() noexcept { return state_buf_[out_idx_]; }


    // state_in, state_out = state_out, state_in
    void SwapStates() noexcept { std::swap(in_idx_, out_idx_); }

    void InitStep() {
        /*
         * TODO:Clean force here.
         */
    };

    void SetGravity(const Vec3& gravity) { model_.gravity_ = gravity; }

    MModel model_;

    static bool RayNormal;

private:
    std::array<State, 2> state_buf_{};
    uint8_t in_idx_  = 0;
    uint8_t out_idx_ = 1;

};

// helper functions for constructing scene
template <class Vec>
size_t checked_index(const int32_t id, const Vec& v, const char* what) {
    if (id < 0) {
        throw std::runtime_error(std::string(what) + " id < 0");
    }
    const auto idx = static_cast<size_t>(id);
    if (idx >= v.size()) {
        throw std::runtime_error(std::string(what) + " id out of range");
    }
    return idx;
}




























#endif //TAIYI_SCENE_H
