//
// Created by tianyan on 1/28/26.
//

#ifndef TAIYI_GEOMETRY_H
#define TAIYI_GEOMETRY_H

#include "Types.h"

enum class GeoType {
    // Plane.
    PLANE = 0,

    // Height field (terrain).
    HFIELD = 1,

    // Sphere.
    SPHERE = 2,

    // Capsule (cylinder with hemispherical ends).
    CAPSULE = 3,

    // Ellipsoid.
    ELLIPSOID = 4,

    // Cylinder.
    CYLINDER = 5,

    // Axis-aligned box.
    BOX = 6,

    // Triangle mesh.
    MESH = 7,

    // Signed distance field.
    SDF = 8,

    // Cone.
    CONE = 9,

    // Convex hull.
    CONVEX_MESH = 10,

    // No geometry (placeholder).
    NONE = 11,

};


struct tetrahedron {
    std::array<VertexID, 4> vertices{0,0,0,0};
    float restVolume{};
    float restSign{};       // +1 or -1
    Mat3 Dm_inv{};

    tetrahedron(VertexID vtex0, VertexID vtex1, VertexID vtex2, VertexID vtex3,
        const Vec3& vtex0_pos, const Vec3& vtex1_pos, const Vec3& vtex2_pos, const Vec3& vtex3_pos);
};

struct triangle {
    std::array<VertexID, 3> vertices{0,0,0};
    float rest_area;
    Mat2 Dm_inv;

    triangle(VertexID vtex0, VertexID vtex1, VertexID vtex2, const Vec3& vtex0_pos, const Vec3& vtex1_pos, const Vec3& vtex2_pos);
};

struct render_trangle {
    std::array<VertexID, 3> vertices{0,0,0};
    render_trangle(const VertexID vtex0, const VertexID vtex1, const VertexID vtex2) :
    vertices{vtex0, vtex1, vtex2} {}
};

struct edge {
    // convention [opp0, opp1, edge_start, edge_end]
    std::array<VertexID, 4> vertices{0,0,0,0};
    float rest_theta;
    float rest_length;
    edge(const VertexID vtex_opp0, const VertexID vtex_opp1, const VertexID vtex_e0, const VertexID vtex_e1,
        const Vec3& vtex_opp0_pos, const Vec3& vtex_opp1_pos, const Vec3& vtex_e0_pos, const Vec3& vtex_e1_pos) :
    vertices{vtex_opp0, vtex_opp1, vtex_e0, vtex_e1} {
        rest_length = (vtex_e1_pos - vtex_e0_pos).norm();
        rest_theta = ComputeRestDihedralAngle(vtex_opp0_pos, vtex_opp1_pos, vtex_e0_pos, vtex_e1_pos);
    };

    static float ComputeRestDihedralAngle(const Vec3& x0, const Vec3& x1, const Vec3& x2, const Vec3& x3);
};


struct TTransform {
    Vec3 p{Vec3::Zero()};          // translation
    Quat q{Quat::Identity()};      // rotation

    static TTransform Identity() { return TTransform{}; }

    [[nodiscard]] Vec3 transformPoint(const Vec3& x) const {
        return q * x + p;
    }
    [[nodiscard]] Vec3 transformVector(const Vec3& v) const {
        return q * v;
    }

    [[nodiscard]] TTransform inverse() const {
        TTransform inv;
        inv.q = q.conjugate();
        inv.p = -(inv.q * p);
        return inv;
    }

    // Compose: this ∘ b  (i.e. apply b then this)
    TTransform operator*(const TTransform& b) const {
        TTransform out;
        out.q = q * b.q;
        out.p = p + q * b.p;
        return out;
    }
};


#endif //TAIYI_GEOMETRY_H