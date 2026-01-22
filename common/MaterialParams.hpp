//
// Created by tianyan on 12/30/25.
//

#ifndef MATERIALPARAMS_HPP
#define MATERIALPARAMS_HPP


#include <stdexcept>
// avoid name conflict with raylib
struct MMaterial {
    [[nodiscard]] float E()      const noexcept { return E_; }
    [[nodiscard]] float nu()     const noexcept { return nu_; }
    [[nodiscard]] float lambda() const noexcept { return lambda_; }
    [[nodiscard]] float mu()     const noexcept { return mu_; }
    [[nodiscard]] float bend_stiff() const noexcept {return bend_stiff_; }

    MMaterial(const float E, const float nu, const float bend_stiff = 1.0f) : E_{E}, nu_{nu}, bend_stiff_(bend_stiff) { update_lambda_mu(); }

    void SetE(float E)  { E_ = E;  update_lambda_mu(); }
    void SetNu(float nu){ nu_ = nu; update_lambda_mu(); }

private:
    float E_{};        // Young's modulus
    float nu_{};       // Poisson's ratio
    float lambda_{};   // first Lamé parameter
    float mu_{};       // second Lamé parameter
    float bend_stiff_{};  // element bending stiffness;

    void update_lambda_mu() {

        if (!(E_ > 0.0f)) throw std::invalid_argument("MMaterial: E must be > 0");
        if (!(nu_ > -1.0f && nu_ < 0.5f)) throw std::invalid_argument("MMaterial: nu must be in (-1, 0.5)");

        const float one_plus_nu = 1.0f + nu_;
        const float one_minus_2nu = 1.0f - 2 * nu_;

        mu_ = E_ / (2.0f * one_plus_nu);
        lambda_ = (E_ * nu_) / (one_plus_nu + one_minus_2nu);
    };

};

inline MMaterial default_cloth() {
    return {2.5e4f, 0.25f, 3.f};
};


inline MMaterial soft_bunny() {
    return {1e2f, 0.3f, 0.0f};
}


#endif //MATERIALPARAMS_HPP