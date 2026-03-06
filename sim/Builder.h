//
// Created by tianyan on 12/23/25.
//

#ifndef TAIYI_BUILDER_H
#define TAIYI_BUILDER_H

#include <optional>
#include <span>

#include "Model.h"
#include "raylib.h"
#include "Types.h"

struct MModel;
struct mesh_on_cpu;

template<class T>
static void ensure_capacity(std::vector<T>& v, size_t extra, const size_t min_grow = 1024) {
    const size_t need = v.size() + extra;
    if (need <= v.capacity()) return;

    size_t new_cap = v.capacity();
    if (new_cap == 0) new_cap = min_grow;

    while (new_cap < need) {
        const size_t prev = new_cap;
        new_cap = new_cap + (new_cap >> 1); // 1.5x
        if (new_cap <= prev) {
            new_cap = need;
            break;
        }
    }
    v.reserve(new_cap);
}

enum class ClothOrientation {
    Vertical,   //  (XY Plane)
    Horizontal  //  (XZ Plane)
};

enum FixSide { NONE = 0, TOP = 1, BOTTOM = 2, LEFT = 4, RIGHT = 8 };

class Builder {

public:
    explicit Builder(MModel& model) : model_(model) {};

    [[nodiscard]] size_t add_cloth(float width, float height, int resX, int resY,
                                   const Vec3 &center = Vec3{0.0f, 0.0f, 0.0f},
                                   float mass = .1f, ClothOrientation orientation = ClothOrientation::Horizontal,
                                   float wave_density = 0.0f, float wave_amp = 0.0f, FixSide fix_mask = FixSide::TOP,
                                   const char * = "cloth") const;

    [[nodiscard]] size_t add_bunny(float height, float mass) const;

    [[nodiscard]] size_t add_single_tet() const;

    [[nodiscard]] size_t add_sphere(float radius, int res, const Vec3& center, float mass, const char* name) const;

    [[nodiscard]] size_t add_link(const std::string &name, const Vec3 &pos, const Quat &rot, bool kinematic = true,
                                  float mass = 0.0f, const Vec3 &com = Vec3::Zero(),
                                  const Mat3 &inertia_tensor = Mat3::Zero()) const;

    [[nodiscard]] size_t add_body(const std::string &name, const Vec3 &pos, const Quat &rot, bool kinematic = true,
                                  float mass = 0.0f, const Vec3 &com = Vec3::Zero(),
                                  const Mat3 &inertia_tensor = Mat3::Zero()) const;

    [[nodiscard]] size_t add_shape_box(size_t body_id, float hx, float hy, float hz, const Vec3 &local_pos = Vec3::Zero(),
                         const Quat &local_rot = Quat::Identity(), float density = -1.0f, float thickness = -1.0f,
                         float margin = -1.0f, bool contribute_mass = true, bool contribute_render_mesh = true) const;

    [[nodiscard]] size_t add_shape_capsule(size_t body_id, float radius, float half_height,
                                           const Vec3 &local_pos = Vec3::Zero(),
                                           const Quat &local_rot = Quat::Identity(), float density = -1.0f,
                                           float thickness = -1.0f, float margin = -1.0f, bool contribute_mass = true,
                                           bool contribute_render_mesh = true) const;

    [[nodiscard]] size_t add_shape_sphere(size_t body_id, float radius, const Vec3 &local_pos = Vec3::Zero(),
                                          const Quat &local_rot = Quat::Identity(), float density = -1.0f,
                                          float thickness = -1.0f, float margin = -1.0f, bool contribute_mass = true,
                                          bool contribute_render_mesh = true) const;

    [[nodiscard]] size_t add_ground_plane() const;

    // --------------- joints ---------------------
    [[nodiscard]] int add_joint(JointType joint_type, int parent, int child, std::span<const JointDofConfig> linear_axes = {},
                     std::span<const JointDofConfig> angular_axes = {},
                     const TTransform &parent_xform = TTransform::Identity(), const TTransform &child_xform = TTransform::Identity(),
                     std::string key = {}, bool collision_filter_parent = true, bool enabled = true) const;

    [[nodiscard]] int add_joint_revolute(int parent, int child, std::optional<Vec3> axis = std::nullopt,
                              const TTransform& parent_xform = TTransform::Identity(),
                              const TTransform& child_xform = TTransform::Identity(),
                              std::optional<float> target_pos = std::nullopt,
                              std::optional<float> target_vel = std::nullopt,
                              std::optional<float> target_ke = std::nullopt,
                              std::optional<float> target_kd = std::nullopt,
                              std::optional<float> limit_lower = std::nullopt,
                              std::optional<float> limit_upper = std::nullopt,
                              std::optional<float> limit_ke = std::nullopt,
                              std::optional<float> limit_kd = std::nullopt,
                              std::optional<float> armature = std::nullopt,
                              std::optional<float> effort_limit = std::nullopt,
                              std::optional<float> velocity_limit = std::nullopt,
                              std::optional<float> friction = std::nullopt, std::string key = {},
                              bool collision_filter_parent = true, bool enabled = true) const;

    [[nodiscard]] int add_joint_free(int child, const TTransform &parent_xform, const TTransform &child_xform, std::string key = {},
                          bool collision_filter_parent = true, bool enabled = true) const;

    [[nodiscard]] int add_joint_fixed(int parent, int child, const TTransform &parent_xform,
                                      const TTransform &child_xform, std::string key = {},
                                      bool collision_filter_parent = true, bool enabled = true);

    [[nodiscard]] int add_joint_d6(int parent, int child, std::span<const JointDofConfig> linear_axes = {},
                                   std::span<const JointDofConfig> angular_axes = {},
                                   const TTransform &parent_xform = TTransform::Identity(),
                                   const TTransform &child_xform = TTransform::Identity(), std::string key = {},
                                   bool collision_filter_parent = true, bool enabled = true) const;

    [[nodiscard]] int add_articulation(std::span<const int> joints, std::string key) const;

    static std::pair<int, int> get_joint_dof_coord_count(JointType joint_type, int num_axes);

    void finalize() const;

private:

    MModel& model_;

    float default_shape_contact_margin = 1e-1f;
    float default_shape_thickness = 1e-5;
    float default_shape_density = 1000.0f;
    float default_shape_ke = 2.5e3f;
    float default_shape_kd = 100.0f;
    float default_shape_kf = 1000.0f;
    float default_friction_mu = 1.0;
    // JointDofConfig default_dof_config = JointDofConfig(Vec3(1.0f,0.0f,0.0f));

private:

    void PrepareCapacity(size_t num) const;

    void AddDeformableBodyInfo(const char* name, size_t n_particle, size_t n_edge,
                size_t n_tri, const size_t n_render_tri, size_t n_tet) const;

    void AddRigidBodyInfo(const char* name, size_t n_vertices, size_t n_render_tris, size_t n_shapes) const;

    static void CheckVertexLimit(const uint32_t local_particle_count) {
        if (local_particle_count > static_cast<size_t>(std::numeric_limits<unsigned short>::max()) + 1ull) {
            throw std::runtime_error("Builder: particle_count > 65536, raylib u16 indices not supported.");
        }
    }

    // helper functions
    static Mat3 parallel_axis(const Vec3& d, const float m) {
        // m * (||d||^2 I - d d^T)
        const float d2 = d.squaredNorm();
        return m * (d2 * Mat3::Identity() - d * d.transpose());
    }

    static bool invert_spd_safe(const Mat3& I, Mat3& outInv) {
        const float det = I.determinant();
        if (std::abs(det) < 1e-12f) return false;
        outInv = I.inverse();
        return true;
    }

    void accumulate_mass_properties(int body_id, float m_add, const Vec3& c_add_body, const Mat3& I_add_about_c_add_body) const;

};



#endif //TAIYI_BUILDER_H