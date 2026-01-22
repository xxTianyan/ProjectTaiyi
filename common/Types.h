//
// Created by 徐天焱 on 2025/11/5.
//

#ifndef TAIYI_TYPES_H
#define TAIYI_TYPES_H

#include <memory>
#include <Eigen/Dense>

struct State;
struct MModel;
class ISample;

using VertexID = size_t;
constexpr VertexID INVALID_VERTEX_ID = 0xFFFF;

using Vec3 = Eigen::Vector3f;
using Vec2 = Eigen::Vector2f;
using Mat3 = Eigen::Matrix3f;
using Mat2 = Eigen::Matrix2f;
using Quat = Eigen::Quaternionf;
using Mat32 = Eigen::Matrix<float, 3, 2>;

using SamplePtr = std::unique_ptr<ISample>;


static int to_int_checked(const size_t v, const char* what) {
    if (v > static_cast<size_t>(std::numeric_limits<int>::max())) {
        throw std::runtime_error(std::string(what) + " too large for int");
    }
    return static_cast<int>(v);
}


#endif //TAIYI_TYPES_H
