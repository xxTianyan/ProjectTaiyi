//
// Created by xumiz on 2026/1/1.
//

#ifndef TAIYI_BASIC_CLOTH_EXAMPLE_H
#define TAIYI_BASIC_CLOTH_EXAMPLE_H

#include "Sample.h"
#include "hanging_cloth.hpp"
#include "Application.h"
#include "Builder.h"
#include "rlgl.h"
#include "Scene.h"
#include "VBDSolver.h"
#include "TriMeshCollision.h"

struct AABBTreeDrawSettings {
    bool enabled = true;
    bool leavesOnly = false;
    int  maxDepth = 32;      // -1 means unlimited
    int  drawEvery = 1;      // 1 means draw all
    bool colorByDepth = true;
};

// Vec3 -> raylib Vector3
inline Vector3 ToRL(const Vec3& v) { return Vector3{v.x(), v.y(), v.z()}; }

//  AABB -> raylib BoundingBox
inline BoundingBox ToBBox(const AABB& b) {
    BoundingBox bb;
    bb.min = ToRL(b.lo);
    bb.max = ToRL(b.hi);
    return bb;
}

inline void DrawAABBWire(const AABB& b, Color c) {
    DrawBoundingBox(ToBBox(b), c);
}

inline Color ColorByDepth(int depth) {
    // 简单做法：深度越深越亮（不指定复杂调色）
    // 注意 raylib Color 分量 0..255
    int v = 80 + (depth * 12);
    if (v > 255) v = 255;
    return Color{static_cast<unsigned char>(v), static_cast<unsigned char>(v), static_cast<unsigned char>(v), 255};
}

inline void DebugDrawAABBTree(const AABBTree& tree, const AABBTreeDrawSettings& s) {
    if (!s.enabled) return;

    int counter = 0;
    tree.traverse([&](int /*nodeId*/, const AABBTree::Node& nd, int depth) {
        if (s.maxDepth >= 0 && depth > s.maxDepth) return;
        if (s.leavesOnly && !nd.is_leaf) return;

        ++counter;
        if (s.drawEvery > 1 && (counter % s.drawEvery) != 0) return;

        Color c = s.colorByDepth ? ColorByDepth(depth) : MAROON;
        DrawAABBWire(nd.box, c);
    });
}


class HangingCloth final : public Sample {

public:

    HangingCloth() {
        max_ticks_per_frame_ = 8;
        substeps_ = 4;
        leaf_size_ = 16;
    }

    void CreateWorld([[maybe_unused]]AppContext &ctx) override {
        MModel model;
        Builder builder(model);
        m_cloth_id_ = builder.add_cloth(2.0f, 3.0f, 16, 24, Vec3{0.0f, 4.0f, 0.0f});
        scene_ = std::make_unique<Scene>(std::move(model));
        solver_ = std::make_unique<VBDSolver>(&scene_->model_, 3, default_cloth());

        // debug aabb tree
        tri_boxes_.resize(scene_->model_.tris.size());
        build(scene_->state_in().particle_pos);
    };

    void Render(AppContext &ctx) override {
        AABBTreeDrawSettings dbgTriTree;
        dbgTriTree.enabled = true;
        dbgTriTree.leavesOnly = false;   // 先画 internal + leaf 看整体分割
        dbgTriTree.maxDepth = -1;        // 不要太大
        dbgTriTree.drawEvery = 1;
        dbgTriTree.colorByDepth = true;


        BeginMode3D(ctx.orbitCam->camera);

        // floor
        if (RenderHelper::IsModelValid(floor_)) {
            DrawModel(floor_, Vector3{0,0,0}, 1.0f, WHITE);
        }

        // scene models
        rlDisableBackfaceCulling();
        renderHelper_.Draw(ctx.is_wire_mode);
        rlEnableBackfaceCulling();

        DebugDrawAABBTree(bvh_tris_, dbgTriTree);   // 你需要给 detector 提供 getter
        // DebugDrawAABBTree(detector.bvh_edges(), dbgEdgeTree);

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

    void OnUpdate([[maybe_unused]]AppContext &ctx) override {
        refit(scene_->state_in().particle_pos);
    }

private:
    size_t m_cloth_id_{};
    std::vector<AABB> tri_boxes_;
    AABBTree bvh_tris_;
    int leaf_size_;

    void compute_tri_aabbs(const std::vector<Vec3>& pos) {
        for (size_t i = 0; i < scene_->model_.tris.size(); ++i) {
            const auto& tv = scene_->model_.render_tris[i].vertices;
            AABB b;
            b.expand(pos[tv[0]]);
            b.expand(pos[tv[1]]);
            b.expand(pos[tv[2]]);
            tri_boxes_[i] = b;
        }
    }

    void build(const std::vector<Vec3>& pos) {
        compute_tri_aabbs(pos);
        bvh_tris_.build(tri_boxes_, leaf_size_);
    }

    void refit(const std::vector<Vec3>& pos) {
        compute_tri_aabbs(pos);
        bvh_tris_.refit(tri_boxes_);
    }


};






#endif //TAIYI_BASIC_CLOTH_EXAMPLE_H