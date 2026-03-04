//
// Created by tianyan on 3/4/26.
//

#ifndef TAIYI_HUMANOID_HPP
#define TAIYI_HUMANOID_HPP

#include "Builder.h"
#include "ISolver.h"
#include "Sample.h"
#include "Scene.h"

namespace {

float deg(float d) {
    return d * 3.14159265358979323846f / 180.0f;
}

// MuJoCo(z-up) -> Taiyi(y-up)
// (x, y, z)_mj -> (x, z, y)_ty
Vec3 mj_to_ty(const Vec3& v) {
    return Vec3(v.x(), v.z(), v.y());
}

// 方向向量同样做坐标轴置换
Vec3 mj_axis_to_ty(const Vec3& v) {
    return Vec3(v.x(), v.z(), v.y());
}

// 把本地 +Y 轴旋到 dir
Quat quat_from_to_y(const Vec3& dir_in) {
    Vec3 from = Vec3::UnitY();
    Vec3 to = dir_in;
    to.normalize();

    float c = from.dot(to);

    if (c > 1.0f - 1.0e-6f) {
        return Quat::Identity();
    }

    if (c < -1.0f + 1.0e-6f) {
        // 180° around X
        return Quat(0.0f, 1.0f, 0.0f, 0.0f);
    }

    Vec3 axis = from.cross(to);
    axis.normalize();

    float angle = std::acos(std::clamp(c, -1.0f, 1.0f));
    float half = 0.5f * angle;
    float s = std::sin(half);
    float w = std::cos(half);

    return Quat(w, axis.x() * s, axis.y() * s, axis.z() * s);
}

// 用 MuJoCo 的 fromto（局部点）加 capsule
 size_t add_capsule_fromto_mj(
    Builder& builder,
    size_t body_id,
    const Vec3& p0_mj,
    const Vec3& p1_mj,
    float radius,
    float density = -1.0f,
    bool contribute_mass = true,
    bool contribute_render_mesh = true)
{
    Vec3 p0 = mj_to_ty(p0_mj);
    Vec3 p1 = mj_to_ty(p1_mj);

    Vec3 d = p1 - p0;
    float len = d.norm();

    Vec3 center = (p0 + p1) * 0.5f;
    Quat q = quat_from_to_y(d);


    float half_height = 0.5f * len;

    return builder.add_shape_capsule(
        body_id,
        radius,
        half_height,
        center,
        q,
        density,
        -1.0f,
        -1.0f,
        contribute_mass,
        contribute_render_mesh
    );
}

 JointDofConfig make_dof_mj(
    const Vec3& axis_mj,
    float lower_rad,
    float upper_rad,
    float limit_ke,
    float limit_kd,
    float target_ke,
    float target_kd,
    float armature = 0.01f,
    float friction = 0.0f)
{
    return JointDofConfig(
        mj_axis_to_ty(axis_mj),
        lower_rad,
        upper_rad,
        limit_ke,
        limit_kd,
        0.0f,   // target_pos
        0.0f,   // target_vel
        target_ke,
        target_kd,
        armature,
        1.0e6f,
        1.0e6f,
        friction
    );
}

// 方便写 joint anchor：输入 MuJoCo 局部点，自动转成 Taiyi(y-up)
 TTransform X_mj(const Vec3& p_mj) {
    TTransform X;
    X.p = mj_to_ty(p_mj);
    X.q = Quat::Identity();
    return X;
}

} // namespace


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

        // MJ local body offsets
        const Vec3 torso_mj       = Vec3( 0.00f,  0.00f,  1.282f);
        const Vec3 waist_lower_mj = Vec3(-0.01f,  0.00f, -0.26f);
        const Vec3 pelvis_mj      = Vec3( 0.00f,  0.00f, -0.165f);

        const Vec3 thigh_r_mj     = Vec3( 0.00f, -0.10f, -0.04f);
        const Vec3 shin_r_mj      = Vec3( 0.00f,  0.01f, -0.40f);
        const Vec3 foot_r_mj      = Vec3( 0.00f,  0.00f, -0.39f);

        const Vec3 thigh_l_mj     = Vec3( 0.00f,  0.10f, -0.04f);
        const Vec3 shin_l_mj      = Vec3( 0.00f, -0.01f, -0.40f);
        const Vec3 foot_l_mj      = Vec3( 0.00f,  0.00f, -0.39f);

        const Vec3 uarm_r_mj      = Vec3( 0.00f, -0.17f,  0.06f);
        const Vec3 larm_r_mj      = Vec3( 0.18f, -0.18f, -0.18f);

        const Vec3 uarm_l_mj      = Vec3( 0.00f,  0.17f,  0.06f);
        const Vec3 larm_l_mj      = Vec3( 0.18f,  0.18f, -0.18f);

        // MJ world body positions
        const Vec3 torso_w_mj       = torso_mj;
        const Vec3 waist_lower_w_mj = torso_w_mj + waist_lower_mj;
        const Vec3 pelvis_w_mj      = waist_lower_w_mj + pelvis_mj;

        const Vec3 thigh_r_w_mj     = pelvis_w_mj + thigh_r_mj;
        const Vec3 shin_r_w_mj      = thigh_r_w_mj + shin_r_mj;
        const Vec3 foot_r_w_mj      = shin_r_w_mj + foot_r_mj;

        const Vec3 thigh_l_w_mj     = pelvis_w_mj + thigh_l_mj;
        const Vec3 shin_l_w_mj      = thigh_l_w_mj + shin_l_mj;
        const Vec3 foot_l_w_mj      = shin_l_w_mj + foot_l_mj;

        const Vec3 uarm_r_w_mj      = torso_w_mj + uarm_r_mj;
        const Vec3 larm_r_w_mj      = uarm_r_w_mj + larm_r_mj;

        const Vec3 uarm_l_w_mj      = torso_w_mj + uarm_l_mj;
        const Vec3 larm_l_w_mj      = uarm_l_w_mj + larm_l_mj;

        // Convert to Taiyi y-up world positions
        const Vec3 torso_w       = mj_to_ty(torso_w_mj);       // (0, 1.282, 0)
        const Vec3 waist_lower_w = mj_to_ty(waist_lower_w_mj); // (-0.01, 1.022, 0)
        const Vec3 pelvis_w      = mj_to_ty(pelvis_w_mj);      // (-0.01, 0.857, 0)

        const Vec3 thigh_r_w     = mj_to_ty(thigh_r_w_mj);     // (-0.01, 0.817, -0.10)
        const Vec3 shin_r_w      = mj_to_ty(shin_r_w_mj);      // (-0.01, 0.417, -0.09)
        const Vec3 foot_r_w      = mj_to_ty(foot_r_w_mj);      // (-0.01, 0.027, -0.09)

        const Vec3 thigh_l_w     = mj_to_ty(thigh_l_w_mj);     // (-0.01, 0.817,  0.10)
        const Vec3 shin_l_w      = mj_to_ty(shin_l_w_mj);      // (-0.01, 0.417,  0.09)
        const Vec3 foot_l_w      = mj_to_ty(foot_l_w_mj);      // (-0.01, 0.027,  0.09)

        const Vec3 uarm_r_w      = mj_to_ty(uarm_r_w_mj);      // (0, 1.342, -0.17)
        const Vec3 larm_r_w      = mj_to_ty(larm_r_w_mj);      // (0.18, 1.162, -0.35)

        const Vec3 uarm_l_w      = mj_to_ty(uarm_l_w_mj);      // (0, 1.342,  0.17)
        const Vec3 larm_l_w      = mj_to_ty(larm_l_w_mj);      // (0.18, 1.162, 0.35)

        // torso
        const size_t torso = builder.add_rigidbody("torso", torso_w, Quat::Identity());
        add_capsule_fromto_mj(builder, torso, Vec3( 0.00f, -0.07f,  0.00f), Vec3( 0.00f,  0.07f,  0.00f), 0.07f);
        add_capsule_fromto_mj(builder, torso, Vec3(-0.01f, -0.06f, -0.12f), Vec3(-0.01f,  0.06f, -0.12f), 0.06f);
        int torso_shape = builder.add_shape_sphere(torso, 0.09f, mj_to_ty(Vec3(0.0f, 0.0f, 0.19f)), Quat::Identity());
        bodies_.push_back(torso);

        // waist / pelvis
        const size_t waist_lower = builder.add_rigidbody("waist_lower", waist_lower_w, Quat::Identity());
        add_capsule_fromto_mj(builder, waist_lower, Vec3( 0.00f, -0.06f, 0.0f), Vec3( 0.00f, 0.06f, 0.0f), 0.06f);
        bodies_.push_back(waist_lower);

        const size_t pelvis = builder.add_rigidbody("pelvis", pelvis_w, Quat::Identity());
        add_capsule_fromto_mj(builder, pelvis,Vec3(-0.02f, -0.07f, 0.0f), Vec3(-0.02f, 0.07f, 0.0f), 0.09f);
        bodies_.push_back(pelvis);

        // right leg
        const size_t thigh_right = builder.add_rigidbody("thigh_right", thigh_r_w, Quat::Identity());
        add_capsule_fromto_mj(builder, thigh_right, Vec3(0, 0.00f, 0.00f), Vec3(0, 0.01f, -0.34f), 0.06f);
        bodies_.push_back(thigh_right);

        const size_t shin_right  = builder.add_rigidbody("shin_right",  shin_r_w,  Quat::Identity());
        add_capsule_fromto_mj(builder, shin_right,  Vec3(0, 0.00f, 0.00f), Vec3(0, 0.00f, -0.30f), 0.049f);
        bodies_.push_back(shin_right);

        const size_t foot_right  = builder.add_rigidbody("foot_right",  foot_r_w,  Quat::Identity());
        add_capsule_fromto_mj(builder, foot_right,  Vec3(-0.07f, -0.01f, 0.0f), Vec3(0.14f, -0.03f, 0.0f), 0.027f);
        add_capsule_fromto_mj(builder, foot_right,  Vec3(-0.07f,  0.01f, 0.0f), Vec3(0.14f,  0.03f, 0.0f), 0.027f);
        bodies_.push_back(foot_right);

        // left leg
        const size_t thigh_left = builder.add_rigidbody("thigh_left", thigh_l_w, Quat::Identity());
        add_capsule_fromto_mj(builder, thigh_left, Vec3(0, 0.00f, 0.00f), Vec3(0, -0.01f, -0.34f), 0.06f);
        bodies_.push_back(thigh_left);

        const size_t shin_left  = builder.add_rigidbody("shin_left",  shin_l_w,  Quat::Identity());
        add_capsule_fromto_mj(builder, shin_left,  Vec3(0, 0.00f, 0.00f), Vec3(0,  0.00f, -0.30f), 0.049f);
        bodies_.push_back(shin_left);

        const size_t foot_left  = builder.add_rigidbody("foot_left",  foot_l_w,  Quat::Identity());
        add_capsule_fromto_mj(builder, foot_left,  Vec3(-0.07f, -0.01f, 0.0f), Vec3(0.14f, -0.03f, 0.0f), 0.027f);
        add_capsule_fromto_mj(builder, foot_left,  Vec3(-0.07f,  0.01f, 0.0f), Vec3(0.14f,  0.03f, 0.0f), 0.027f);
        bodies_.push_back(foot_left);

        // right arm
        const size_t upper_arm_right = builder.add_rigidbody("upper_arm_right", uarm_r_w, Quat::Identity());
        add_capsule_fromto_mj(builder, upper_arm_right, Vec3(0, 0, 0), Vec3(0.16f, -0.16f, -0.16f), 0.04f);
        bodies_.push_back(upper_arm_right);

        const size_t lower_arm_right = builder.add_rigidbody("lower_arm_right", larm_r_w, Quat::Identity());
        add_capsule_fromto_mj(builder, lower_arm_right, Vec3(0.01f, 0.01f, 0.01f), Vec3(0.17f, 0.17f, 0.17f), 0.031f);
        int lar_shape = builder.add_shape_sphere(lower_arm_right, 0.04f, mj_to_ty(Vec3(0.18f, 0.18f, 0.18f)), Quat::Identity());
        bodies_.push_back(lower_arm_right);

        // left arm
        const size_t upper_arm_left = builder.add_rigidbody("upper_arm_left", uarm_l_w, Quat::Identity());
        add_capsule_fromto_mj(builder, upper_arm_left, Vec3(0, 0, 0), Vec3(0.16f, 0.16f, -0.16f), 0.04f);
        bodies_.push_back(upper_arm_left);

        const size_t lower_arm_left = builder.add_rigidbody("lower_arm_left", larm_l_w, Quat::Identity());
        add_capsule_fromto_mj(builder, lower_arm_left, Vec3(0.01f, -0.01f, 0.01f), Vec3(0.17f, -0.17f, 0.17f), 0.031f);
        int lal_shape = builder.add_shape_sphere(lower_arm_left, 0.04f, mj_to_ty(Vec3(0.18f, -0.18f, 0.18f)), Quat::Identity());
        bodies_.push_back(lower_arm_left);

        std::vector<int> joints;
        joints.reserve(16);

        // root free joint
        joints.push_back(builder.add_joint_free(
            static_cast<int>(torso),
            TTransform::Identity(),
            TTransform::Identity(),
            "root"
        ));

        // torso -> waist_lower : abdomen_z + abdomen_y
        /*{
            std::array<JointDofConfig, 2> axes = {
                make_dof_mj(Vec3(0, 0, 1), deg(-45), deg(45), 1.0e4f, 10.0f, 20.0f, 5.0f),
                make_dof_mj(Vec3(0, 1, 0), deg(-75), deg(30), 1.0e4f, 10.0f, 10.0f, 5.0f)
            };

            joints.push_back(builder.add_joint_d6(
                static_cast<int>(torso),
                static_cast<int>(waist_lower),
                {},
                axes,
                X_mj(Vec3(-0.01f, 0.0f, -0.195f)), // torso local
                X_mj(Vec3( 0.00f, 0.0f,  0.065f)), // waist_lower local
                "abdomen_zy"
            ));
        }

        // waist_lower -> pelvis : abdomen_x
        joints.push_back(builder.add_joint_revolute(
            static_cast<int>(waist_lower),
            static_cast<int>(pelvis),
            mj_axis_to_ty(Vec3(1, 0, 0)),
            X_mj(Vec3(0.0f, 0.0f, -0.065f)),
            X_mj(Vec3(0.0f, 0.0f,  0.10f)),
            0.0f, 0.0f,
            10.0f, 5.0f,
            deg(-35), deg(35),
            1.0e4f, 10.0f,
            0.01f,
            1.0e6f, 1.0e6f,
            0.0f,
            "abdomen_x"
        ));

        // pelvis -> thigh_right : 3-DoF hip
        {
            std::array<JointDofConfig, 3> axes = {
                make_dof_mj(Vec3(1, 0, 0), deg(-30),  deg(10), 1.0e4f, 10.0f, 10.0f, 5.0f),
                make_dof_mj(Vec3(0, 0, 1), deg(-60),  deg(35), 1.0e4f, 10.0f, 10.0f, 5.0f),
                make_dof_mj(Vec3(0, 1, 0), deg(-150), deg(20), 1.0e4f, 10.0f, 10.0f, 5.0f)
            };

            joints.push_back(builder.add_joint_d6(
                static_cast<int>(pelvis),
                static_cast<int>(thigh_right),
                {},
                axes,
                X_mj(Vec3(0.0f, -0.10f, -0.04f)),
                TTransform::Identity(),
                "hip_right"
            ));
        }

        // thigh_right -> shin_right : knee_right
        joints.push_back(builder.add_joint_revolute(
            static_cast<int>(thigh_right),
            static_cast<int>(shin_right),
            mj_axis_to_ty(Vec3(0, -1, 0)),
            X_mj(Vec3(0.0f, 0.01f, -0.38f)),
            X_mj(Vec3(0.0f, 0.00f,  0.02f)),
            0.0f, 0.0f,
            1.0f, 0.2f,
            deg(-160), deg(2),
            1.0e4f, 10.0f,
            0.01f,
            1.0e6f, 1.0e6f,
            0.0f,
            "knee_right"
        ));

        // shin_right -> foot_right : ankle_y + ankle_x (近似共点于 ankle_y 的 pos)
        {
            std::array<JointDofConfig, 2> axes = {
                make_dof_mj(Vec3(0, 1, 0),   deg(-50), deg(50), 1.0e4f, 10.0f, 6.0f, 0.2f),
                make_dof_mj(Vec3(1, 0, 0.5), deg(-50), deg(50), 1.0e4f, 10.0f, 3.0f, 0.2f)
            };

            joints.push_back(builder.add_joint_d6(
                static_cast<int>(shin_right),
                static_cast<int>(foot_right),
                {},
                axes,
                X_mj(Vec3(0.0f, 0.0f, -0.31f)),
                X_mj(Vec3(0.0f, 0.0f,  0.08f)),
                "ankle_right_approx"
            ));
        }

        // pelvis -> thigh_left : 3-DoF hip
        {
            std::array<JointDofConfig, 3> axes = {
                make_dof_mj(Vec3(-1, 0, 0), deg(-30),  deg(10), 1.0e4f, 10.0f, 10.0f, 5.0f),
                make_dof_mj(Vec3( 0, 0,-1), deg(-60),  deg(35), 1.0e4f, 10.0f, 10.0f, 5.0f),
                make_dof_mj(Vec3( 0, 1, 0), deg(-150), deg(20), 1.0e4f, 10.0f, 10.0f, 5.0f)
            };

            joints.push_back(builder.add_joint_d6(
                static_cast<int>(pelvis),
                static_cast<int>(thigh_left),
                {},
                axes,
                X_mj(Vec3(0.0f, 0.10f, -0.04f)),
                TTransform::Identity(),
                "hip_left"
            ));
        }

        // thigh_left -> shin_left : knee_left
        joints.push_back(builder.add_joint_revolute(
            static_cast<int>(thigh_left),
            static_cast<int>(shin_left),
            mj_axis_to_ty(Vec3(0, -1, 0)),
            X_mj(Vec3(0.0f, -0.01f, -0.38f)),
            X_mj(Vec3(0.0f,  0.00f,  0.02f)),
            0.0f, 0.0f,
            1.0f, 0.2f,
            deg(-160), deg(2),
            1.0e4f, 10.0f,
            0.01f,
            1.0e6f, 1.0e6f,
            0.0f,
            "knee_left"
        ));

        // shin_left -> foot_left : ankle_y + ankle_x (近似)
        {
            std::array<JointDofConfig, 2> axes = {
                make_dof_mj(Vec3( 0, 1,  0),   deg(-50), deg(50), 1.0e4f, 10.0f, 6.0f, 0.2f),
                make_dof_mj(Vec3(-1, 0, -0.5), deg(-50), deg(50), 1.0e4f, 10.0f, 3.0f, 0.2f)
            };

            joints.push_back(builder.add_joint_d6(
                static_cast<int>(shin_left),
                static_cast<int>(foot_left),
                {},
                axes,
                X_mj(Vec3(0.0f, 0.0f, -0.31f)),
                X_mj(Vec3(0.0f, 0.0f,  0.08f)),
                "ankle_left_approx"
            ));
        }

        // torso -> upper_arm_right : 2-DoF shoulder
        {
            std::array<JointDofConfig, 2> axes = {
                make_dof_mj(Vec3(2,  1,  1), deg(-85), deg(60), 1.0e4f, 10.0f, 1.0f, 0.2f),
                make_dof_mj(Vec3(0, -1,  1), deg(-85), deg(60), 1.0e4f, 10.0f, 1.0f, 0.2f)
            };

            joints.push_back(builder.add_joint_d6(
                static_cast<int>(torso),
                static_cast<int>(upper_arm_right),
                {},
                axes,
                X_mj(Vec3(0.0f, -0.17f, 0.06f)),
                TTransform::Identity(),
                "shoulder_right"
            ));
        }

        // upper_arm_right -> lower_arm_right : elbow_right
        joints.push_back(builder.add_joint_revolute(
            static_cast<int>(upper_arm_right),
            static_cast<int>(lower_arm_right),
            mj_axis_to_ty(Vec3(0, -1, 1)),
            X_mj(Vec3(0.18f, -0.18f, -0.18f)),
            TTransform::Identity(),
            0.0f, 0.0f,
            0.0f, 0.2f,
            deg(-100), deg(50),
            1.0e4f, 10.0f,
            0.01f,
            1.0e6f, 1.0e6f,
            0.0f,
            "elbow_right"
        ));

        // torso -> upper_arm_left : 2-DoF shoulder
        {
            std::array<JointDofConfig, 2> axes = {
                make_dof_mj(Vec3(-2, 1, -1), deg(-85), deg(60), 1.0e4f, 10.0f, 1.0f, 0.2f),
                make_dof_mj(Vec3( 0,-1, -1), deg(-85), deg(60), 1.0e4f, 10.0f, 1.0f, 0.2f)
            };

            joints.push_back(builder.add_joint_d6(
                static_cast<int>(torso),
                static_cast<int>(upper_arm_left),
                {},
                axes,
                X_mj(Vec3(0.0f, 0.17f, 0.06f)),
                TTransform::Identity(),
                "shoulder_left"
            ));
        }

        // upper_arm_left -> lower_arm_left : elbow_left
        joints.push_back(builder.add_joint_revolute(
            static_cast<int>(upper_arm_left),
            static_cast<int>(lower_arm_left),
            mj_axis_to_ty(Vec3(0, -1, -1)),
            X_mj(Vec3(0.18f, 0.18f, -0.18f)),
            TTransform::Identity(),
            0.0f, 0.0f,
            0.0f, 0.2f,
            deg(-100), deg(50),
            1.0e4f, 10.0f,
            0.01f,
            1.0e6f, 1.0e6f,
            0.0f,
            "elbow_left"
        ));*/


        // articulation + finalize
        const int humanoid_art = builder.add_articulation(joints, "humanoid_v1_yup");
        builder.finalize();

        scene_ = std::make_unique<Scene>(std::move(model));
        dbg_ = std::make_unique<SolverDebugger>();
        solver_ = std::make_unique<OrderSolver>(scene_->model_);

    }

    void BindShaders(AppContext &ctx) override {
        ctx.shader_manager->LoadShaderProgram("rubber", "../resources/shaders/rubber.vs", "../resources/shaders/rubber.fs");
        const auto rubber_shader = ctx.shader_manager->Get("rubber")->shader;
        ShaderManager::BindMatrices(rubber_shader);
        ShaderManager::SetCommonShaderParams(rubber_shader);
        rubber_shader.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(rubber_shader, "texture0");

        for (auto b : bodies_) {
            auto model_ = renderHelper_.GetRLModel_r(b);
            model_.materials[0].shader = rubber_shader;
            model_.materials[0].maps[MATERIAL_MAP_ALBEDO].color = Color{230, 200, 160, 255};
        }

    }

private:
    std::vector<int> bodies_;
    size_t shape_ground_plane_{};

};





#endif //TAIYI_HUMANOID_HPP