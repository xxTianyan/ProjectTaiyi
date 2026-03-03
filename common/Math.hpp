//
// Created by tianyan on 1/4/26.
//

#ifndef TAIYI_MATH_HPP
#define TAIYI_MATH_HPP
#include "Types.h"

namespace TY {

    [[nodiscard]] inline Mat3 Skew(const Vec3& v) {
    Mat3 m;
    m <<  0.0f,   -v.z(),  v.y(),
          v.z(),   0.0f,  -v.x(),
         -v.y(),   v.x(),  0.0f;
    return m;
    }

    [[nodiscard]] inline Mat3 ComputeNormalizedVectorDerivative(
        const float unnormalized_vec_length,
        const Vec3& normalized_vec,
        const Mat3& unnormalized_vec_derivative) {

        constexpr float eps = 1.0e-8f;
        if (unnormalized_vec_length < eps) {
            return Mat3::Zero();
        }

        const Mat3 I = Mat3::Identity();
        const Mat3 projection = I - (normalized_vec * normalized_vec.transpose()); // outer

        return (1.0f / unnormalized_vec_length) * projection * unnormalized_vec_derivative;
    }

    [[nodiscard]] inline Vec3 ComputeAngleDerivative(
    const Vec3& n1_hat,
    const Vec3& n2_hat,
    const Vec3& e_hat,
    const Mat3& dn1hat_dx,
    const Mat3& dn2hat_dx,
    const float sin_theta,
    const float cos_theta,
    const Mat3& skew_n1,
    const Mat3& skew_n2) {

        const Vec3 dsin_dx = (skew_n1 * dn2hat_dx - skew_n2 * dn1hat_dx).transpose() * e_hat;

        const Vec3 dcos_dx = dn1hat_dx.transpose() * n2_hat + dn2hat_dx.transpose() * n1_hat;

        return dsin_dx * cos_theta - dcos_dx * sin_theta;
    }

    inline Vec3 SolveSPDOrRegularize(Mat3& H, const Vec3& f) {
        using Scalar = Mat3::Scalar;

        // 先做对称化（用临时，避免引用 alias）
        H = Scalar(0.5) * (H + H.transpose());

        // finite check（强烈建议）
        if (!H.allFinite() || !f.allFinite()) {
            return Vec3::Zero();
        }

        Eigen::SelfAdjointEigenSolver<Mat3> es(H);
        if (es.info() != Eigen::Success) {
            return Vec3::Zero();
        }

        Eigen::Matrix<Scalar,3,1> d = es.eigenvalues();
        const Mat3& Q = es.eigenvectors();

        // 选择一个更稳的 tau：与矩阵谱/尺度相关
        Scalar dmax = d.cwiseAbs().maxCoeff();
        Scalar tau  = Scalar(1e-6) * std::max(Scalar(1), dmax);  // 你可改 1e-5/1e-4 更稳

        // clamp eigenvalues to be >= tau (SPD)
        d = d.cwiseMax(tau);

        // solve: x = Q * diag(1/d) * Q^T * f
        Eigen::Matrix<Scalar,3,1> rhs = f;
        Eigen::Matrix<Scalar,3,1> y = Q.transpose() * rhs;
        y.array() /= d.array();
        Vec3 x = Q * y;

        if (!x.allFinite()) {
            return Vec3::Zero();
        }
        return x;
    }

    inline float norm_huber(const  Vec3& v, const float delta) {
        auto a = v.dot(v);
        if (a <= delta * delta)
            return a * 0.5;
        return delta * (std::sqrt(a) - 0.5f * delta);
    }

    static  Quat quat_from_angular_velocity(const Vec3& w) {
        return Quat{w.z(), w.x(), w.y(), 0.0f};
    }

}



#endif //TAIYI_MATH_HPP