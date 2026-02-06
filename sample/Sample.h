//
// Created by xumiz on 2025/12/24.
//

#ifndef TAIYI_SAMPLE_H
#define TAIYI_SAMPLE_H

#include <memory>
#include <raylib.h>

#include "Debugger.hpp"
#include "ISample.h"
#include "RenderHelper.h"
#include "DbgUI.hpp"

class Scene;
class VBDSolver;
struct AppContext;


struct SimContext {
    float dt{};
    size_t frame_id{};
};

class Sample : public ISample {
public:
    Sample() = default;
    ~Sample() override = default;

    // provide sample name
    [[nodiscard]] const char* Name() const override { return "Empty Scene";};

    void OnEnter(AppContext& ctx) override;

    void OnExit(AppContext& ctx) override;

    void Update(AppContext& ctx) override;

    void Render(AppContext& ctx) override;

    void DrawUI(AppContext& ctx) override;

    void Reset(AppContext& ctx) override;

    virtual void CreateFloor(AppContext& ctx);

    // api functions that need to be over-ride when inherited
    virtual void CreateWorld(AppContext& ctx);
    virtual void Step([[maybe_unused]]float dt);
    virtual void BindShaders([[maybe_unused]]AppContext& ctx) {};

    // call back function for updating
    virtual void OnUpdate([[maybe_unused]]AppContext& ctx) {};

    // clean cpu resource
    virtual void CleanUp();

protected:

    virtual void BuildRenderResources();
    // clean gpu resource
    virtual void DestroyRenderResources();

    float sim_accum_ = 0.0f;

    float fixed_dt0_ = 1.0 / 60.0f;
    float fixed_dt_  = fixed_dt0_;  // physical time step

    int max_ticks_per_frame0_ = 2;
    int max_ticks_per_frame_  = max_ticks_per_frame0_;

    int substeps0_ = 8;
    int substeps_  = substeps0_;              // step in ticks

    // for simulation, remember to initialize
    std::unique_ptr<Scene> scene_;
    // need to change to ISolver
    std::unique_ptr<VBDSolver> solver_;
    // debugger
    std::unique_ptr<SolverDebugger> dbg_;

    // for rendering
    RenderHelper renderHelper_;
    SolverDebuggerUI dbg_ui_;
    Model floor_{};

};




#endif //TAIYI_SAMPLE_H