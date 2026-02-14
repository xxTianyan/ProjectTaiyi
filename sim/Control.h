//
// Created by tianyan on 2/10/26.
//

#ifndef TAIYI_CONTROL_H
#define TAIYI_CONTROL_H

#include <vector>
#include "Types.h"

struct Control {
    std::vector<Vec3> joint_target_pos;
    std::vector<Vec3> joint_target_vel;
    std::vector<float> joint_f;

};







#endif //TAIYI_CONTROL_H