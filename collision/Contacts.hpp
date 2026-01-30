//
// Created by tianyan on 1/5/26.
//

#ifndef TAIYI_CONTACTS_H
#define TAIYI_CONTACTS_H


#include <vector>
#include "Types.h"

struct Contacts {

    // configuration
    bool per_contact_shape_properties{false};
    bool clear_buffers{false};
    bool requires_grad{false};

    std::int32_t rigid_contact_max{0};
    std::int32_t soft_contact_max{0};

    // consolidated counter array: [rigid_count, soft_count]
    // Newton: wp.zeros(2, int32) + slices
    std::int32_t counter_[2]{0, 0};

    // handy accessors
    [[nodiscard]] std::int32_t& rigid_contact_count() noexcept { return counter_[0]; }
    [[nodiscard]] std::int32_t  rigid_contact_count() const noexcept { return counter_[0]; }
    [[nodiscard]] std::int32_t& soft_contact_count() noexcept { return counter_[1]; }
    [[nodiscard]] std::int32_t  soft_contact_count() const noexcept { return counter_[1]; }

    // rigid contacts (shape-shape)
    // manifold: shape / point / offset/ normal / thickness
    std::vector<std::int32_t> rigid_contact_point_id;
    std::vector<std::int32_t> rigid_contact_shape0;
    std::vector<std::int32_t> rigid_contact_shape1;

    std::vector<Vec3>  rigid_contact_point0;
    std::vector<Vec3>  rigid_contact_point1;
    std::vector<Vec3>  rigid_contact_offset0;
    std::vector<Vec3>  rigid_contact_offset1;
    std::vector<Vec3>  rigid_contact_normal;

    std::vector<float> rigid_contact_thickness0;
    std::vector<float> rigid_contact_thickness1;

    // per-contact shape properties (optional)
    /*std::vector<float> rigid_contact_stiffness; // ke
    std::vector<float> rigid_contact_damping;   // kd
    std::vector<float> rigid_contact_friction;  // mu*/

    // soft contacts (particle-shape)
    std::vector<std::int32_t> soft_contact_particle;
    std::vector<std::int32_t> soft_contact_shape;

    std::vector<Vec3> soft_contact_body_pos;
    std::vector<Vec3> soft_contact_body_vel;
    std::vector<Vec3> soft_contact_normal;

    // ----------------------------
    // ctor / init
    // ----------------------------
    Contacts() = default;

    Contacts(const std::int32_t rigidMax,
             const std::int32_t softMax,
             const bool requiresGrad = false,
             const bool perContactProps = false,
             const bool clearBuffers = false)
    {
        init(rigidMax, softMax, requiresGrad, perContactProps, clearBuffers);
    }

    void init(const std::int32_t rigidMax,
              const std::int32_t softMax,
              const bool requiresGrad = false,
              const bool perContactProps = false,
              const bool clearBuffers = true)
    {
        if (rigidMax < 0 || softMax < 0)
            throw std::runtime_error("Contacts::init: max < 0");

        requires_grad = requiresGrad;
        per_contact_shape_properties = perContactProps;
        clear_buffers = clearBuffers;

        rigid_contact_max = rigidMax;
        soft_contact_max  = softMax;

        // allocate buffers with Newton-like defaults:
        // - shapes params default to -1
        // - vectors default to 0
        rigid_contact_point_id.assign(rigid_contact_max, 0);
        rigid_contact_shape0.assign(rigid_contact_max, -1);
        rigid_contact_shape1.assign(rigid_contact_max, -1);

        rigid_contact_point0.assign(rigid_contact_max, Vec3::Zero());
        rigid_contact_point1.assign(rigid_contact_max, Vec3::Zero());
        rigid_contact_offset0.assign(rigid_contact_max, Vec3::Zero());
        rigid_contact_offset1.assign(rigid_contact_max, Vec3::Zero());
        rigid_contact_normal.assign(rigid_contact_max, Vec3::Zero());

        rigid_contact_thickness0.assign(rigid_contact_max, 0.0f);
        rigid_contact_thickness1.assign(rigid_contact_max, 0.0f);


        /*if (per_contact_shape_properties) {
            rigid_contact_stiffness.assign(rigid_contact_max, 0.0f);
            rigid_contact_damping.assign(rigid_contact_max, 0.0f);
            rigid_contact_friction.assign(rigid_contact_max, 0.0f);
        } else {
            rigid_contact_stiffness.clear();
            rigid_contact_damping.clear();
            rigid_contact_friction.clear();
        }*/

        soft_contact_particle.assign(soft_contact_max, -1);
        soft_contact_shape.assign(soft_contact_max, -1);

        soft_contact_body_pos.assign(soft_contact_max, Vec3::Zero());
        soft_contact_body_vel.assign(soft_contact_max, Vec3::Zero());
        soft_contact_normal.assign(soft_contact_max, Vec3::Zero());

        clear();
    }

    // ------clear ---------
    void clear() {
        // Newton: _counter_array.zero_() (single kernel)
        counter_[0] = 0;
        counter_[1] = 0;

        if (!clear_buffers) {
            // Optimized path:
            // collision writes [0, count) and solver reads [0, count)
            return;
        }

        // Conservative path: fill sentinel values / zeros
        std::ranges::fill(rigid_contact_shape0, -1);
        std::ranges::fill(rigid_contact_shape1, -1);

        /*
        if (per_contact_shape_properties) {
            std::ranges::fill(rigid_contact_stiffness, 0.0f);
            std::ranges::fill(rigid_contact_damping,   0.0f);
            std::ranges::fill(rigid_contact_friction,  0.0f);
        }
        */

        std::ranges::fill(soft_contact_particle, -1);
        std::ranges::fill(soft_contact_shape,    -1);

    }


    // helpers (CPU append; GPU would use atomicAdd)
    [[nodiscard]] bool has_room_for_rigid(std::int32_t n = 1) const noexcept {
        return rigid_contact_count() + n <= rigid_contact_max;
    }
    [[nodiscard]] bool has_room_for_soft(std::int32_t n = 1) const noexcept {
        return soft_contact_count() + n <= soft_contact_max;
    }

    // Returns slot index or -1 if overflow
    std::int32_t alloc_rigid_slot() noexcept {
        const auto c = rigid_contact_count();
        if (c >= rigid_contact_max) return -1;
        counter_[0] = c + 1;
        return c;
    }

    std::int32_t alloc_soft_slot() noexcept {
        const auto c = soft_contact_count();
        if (c >= soft_contact_max) return -1;
        counter_[1] = c + 1;
        return c;
    }
};

#endif //TAIYI_CONTACTS_H