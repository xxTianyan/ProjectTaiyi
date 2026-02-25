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

#endif //TAIYI_JOINTS_H