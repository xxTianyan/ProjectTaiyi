//
// Created by tianyan on 3/4/26.
//

#ifndef TAIYI_HUMANOID_HPP
#define TAIYI_HUMANOID_HPP

#include "Builder.h"
#include "ISolver.h"
#include "Sample.h"
#include "Scene.h"

class Humanoid final : public Sample {

public:

    Humanoid() {
        max_ticks_per_frame0_ = 2;
        substeps0_ = 12;
    }

    void Step(float dt) override {
        if (scene_ == nullptr) return;
        if (solver_ == nullptr) return;

        // collide here
        const Contacts* contacts = scene_->model_.collide(scene_->state_out());

        solver_->Step(scene_->state_in(), scene_->state_out(), contacts, dt);
        scene_->SwapStates();
    };

    void CreateWorld(AppContext &ctx) override {
        MModel model;
        Builder builder(model);
        shape_ground_plane_ = builder.add_ground_plane();

    }

    void BindShaders(AppContext &ctx) override {


    }

private:
    std::vector<int> bodies_;
    std::vector<int> joints_;
    size_t shape_ground_plane_{};

};





#endif //TAIYI_HUMANOID_HPP