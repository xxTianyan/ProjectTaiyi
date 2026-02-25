//
// Created by tianyan on 2/25/26.
//

#ifndef TAIYI_JOINTS_H
#define TAIYI_JOINTS_H

enum class JointType {

    /*Enumeration of joint types supported in Newton.*/

    PRISMATIC = 0,
    /*Prismatic joint: allows translation along a single axis (1 DoF).*/

    REVOLUTE = 1,
    /*Revolute joint: allows rotation about a single axis (1 DoF).*/

    BALL = 2,
    /*Ball joint: allows rotation about all three axes (3 DoF, quaternion parameterization).*/

    FIXED = 3,
    /*Fixed joint: locks all relative motion (0 DoF).*/

    FREE = 4,
    /*Free joint: allows full 6-DoF motion (translation and rotation, 7 coordinates).*/

    DISTANCE = 5,
    /*Distance joint: keeps two bodies at a distance within its joint limits (6 DoF, 7 coordinates).*/

    D6 = 6,
    /*6-DoF joint: Generic joint with up to 3 translational and 3 rotational degrees of freedom.*/

    CABLE = 7,
    /*Cable joint: one linear (stretch) and one angular (isotropic bend/twist) DoF.*/
};

constexpr float MAXVAL = 1.0e30f;
constexpr float EPS_NORM = 1.0e-12f;

struct JointDofConfig {
    Vec3  axis;             // normalized, in joint parent anchor frame
    float limit_lower   = -MAXVAL;
    float limit_upper   =  MAXVAL;
    float limit_ke      =  1.0e4f;
    float limit_kd      =  1.0e1f;

    float target_pos    =  0.0f;
    float target_vel    =  0.0f;
    float target_ke     =  0.0f;
    float target_kd     =  0.0f;

    float armature      =  1.0e-2f;
    float effort_limit  =  1.0e6f;
    float velocity_limit=  1.0e6f;
    float friction      =  0.0f;

    explicit JointDofConfig(
        const Vec3 &axis_vec,
        const float limit_lower_ = -MAXVAL,
        const float limit_upper_ =  MAXVAL,
        const float limit_ke_    =  1.0e4f,
        const float limit_kd_    =  1.0e1f,
        const float target_pos_  =  0.0f,
        const float target_vel_  =  0.0f,
        const float target_ke_   =  0.0f,
        const float target_kd_   =  0.0f,
        const float armature_    =  1.0e-2f,
        const float effort_limit_=  1.0e6f,
        const float velocity_limit_=1.0e6f,
        const float friction_    =  0.0f
    )
    : axis(axis_vec)
    , limit_lower(limit_lower_)
    , limit_upper(limit_upper_)
    , limit_ke(limit_ke_)
    , limit_kd(limit_kd_)
    , target_pos(target_pos_)
    , target_vel(target_vel_)
    , target_ke(target_ke_)
    , target_kd(target_kd_)
    , armature(armature_)
    , effort_limit(effort_limit_)
    , velocity_limit(velocity_limit_)
    , friction(friction_) {
        fix_target_pos_if_outside_limits();
        axis.normalize();
    }

private:
    void fix_target_pos_if_outside_limits() {
        if (target_pos > limit_upper || target_pos < limit_lower) {
            target_pos = 0.5f * (limit_lower + limit_upper);
        }
    }
};

#endif //TAIYI_JOINTS_H