//
// Created by tianyan on 1/28/26.
//

#include "Geometry.h"

tetrahedron::tetrahedron(const VertexID vtex0, const VertexID vtex1, const VertexID vtex2, const VertexID vtex3,
                         const Vec3 &vtex0_pos, const Vec3 &vtex1_pos, const Vec3 &vtex2_pos,
                         const Vec3 &vtex3_pos) : vertices{vtex0, vtex1, vtex2, vtex3} {
    // construct Dm
    const Vec3 e1 = vtex1_pos - vtex0_pos;
    const Vec3 e2 = vtex2_pos - vtex0_pos;
    const Vec3 e3 = vtex3_pos - vtex0_pos;

    Mat3 Dm;
    Dm.col(0) = e1;
    Dm.col(1) = e2;
    Dm.col(2) = e3;

    // more stable det：det = dot(e1, cross(e2, e3))
    const auto detDm = e1.dot(e2.cross(e3));
    restSign = detDm > 0 ? 1.0f : -1.0f;

    const float absDet = std::fabs(detDm);

    // check if degenerate
    if (constexpr float kEps = 1.0e-12f; absDet < kEps)
        throw std::runtime_error("tetrahedron::compute_rest: degenerate tetrahedron (|det(Dm)| too small).");

    restVolume = absDet * (1.0f / 6.0f);

    // pre-compute
    Dm_inv  = Dm.inverse();
};

triangle::triangle(VertexID vtex0, VertexID vtex1, VertexID vtex2, const Vec3 &vtex0_pos, const Vec3 &vtex1_pos,
                   const Vec3 &vtex2_pos) : vertices{vtex0, vtex1, vtex2} {

    // build local 2d material basis from rest position
    const Vec3 e01 = vtex1_pos - vtex0_pos;
    const float L = e01.norm();
    constexpr float eps = 1e-8f;

    if (L < eps) {
        // Degenerate: vtex0_pos == vtex1_pos
        rest_area = 0.0f;
        Dm_inv.setZero();
        return;
    }

    const Vec3 e1 = e01 / L;

    const Vec3 e02 = vtex2_pos - vtex0_pos;
    Vec3 n = e01.cross(e02);
    const float n_len = n.norm();
    if (n_len < eps) {
        // Degenerate: collinear triangle in rest pose
        rest_area = 0.0f;
        Dm_inv.setZero();
        return;
    }

    n /= n_len;
    const Vec3 e2 = n.cross(e1);  // orthonormal within triangle plane

    // --- Rest 2D coordinates ---
    // u0 = (0,0)
    // u1 = (L,0)
    // u2 = (u,v)
    const float u = e02.dot(e1);
    const float v = e02.dot(e2);

    // Dm = [[L, u],
    //       [0, v]]
    // det(Dm) = L*v
    const float det = L * v;
    if (det < eps) {
        rest_area = 0.0f;
        Dm_inv.setZero();
        return;
    }

    // --- rest area ---
    rest_area = 0.5f * det;


    // --- explicit inverse of upper-triangular Dm ---
    // Dm^{-1} = [[ 1/L,   -u/(L*v)],
    //           [ 0,      1/v      ]]
    Dm_inv(0, 0) = 1.0f / L;
    Dm_inv(0, 1) = -u / det;  // -u/(L*v)
    Dm_inv(1, 0) = 0.0f;
    Dm_inv(1, 1) = 1.0f / v;
}


float edge::ComputeRestDihedralAngle(const Vec3 &x0, const Vec3 &x1, const Vec3 &x2, const Vec3 &x3) {

    constexpr float eps = 1e-6f;
    const Vec3 e = x3 - x2;
    const float e_norm = e.norm();
    if (e_norm < eps) return 0.0f;

    const Vec3 x02 = x2 - x0;
    const Vec3 x03 = x3 - x0;
    const Vec3 x12 = x2 - x1;
    const Vec3 x13 = x3 - x1;

    const Vec3 n1 = x02.cross(x03);
    const Vec3 n2 = x13.cross(x12);

    const float n1_norm = n1.norm();
    const float n2_norm = n2.norm();

    if (n1_norm < eps || n2_norm < eps) return 0.0f;

    const Vec3 n1_hat = n1 / n1_norm;
    const Vec3 n2_hat = n2 / n2_norm;
    const Vec3 e_hat = e / e_norm;

    const float sin_theta = (n1_hat.cross(n2_hat)).dot(e_hat);
    const float cos_theta = n1_hat.dot(n2_hat);

    return std::atan2(sin_theta, cos_theta);
}

GeoData GeoData::create_geo_data(const int shape_index, const Vec3 &shape_pos, const Quat &shape_rot,
                                 const GeoType shape_type, const int body_index, const Vec3 &body_pos,
                                 const Quat &body_rot, const Vec3 &shape_scale, const float thickness) {

    GeoData g;
    g.shape_index = shape_index;
    g.rigid_body_index = body_index;

    // body -> world
    if (g.rigid_body_index >= 0) {
        g.X_wb.p = body_pos;
        g.X_wb.q = body_rot;
    }else {
        g.X_wb = TTransform::Identity();
    }

    g.X_bw = g.X_wb.inverse();

    // shape -> body
    g.X_bs.p = shape_pos;
    g.X_bs.q = shape_rot;

    // shape -> world
    g.X_ws = g.X_wb * g.X_bs;
    g.X_sw = g.X_ws.inverse();

    // geometry props
    g.geo_type = shape_type;
    g.geo_scale = shape_scale;
    g.min_scale = std::min({g.geo_scale.x(), g.geo_scale.y(), g.geo_scale.z()});
    g.thickness = thickness;

    g.radius_eff = 0.0f;

    // future expand：only Sphere/Capsule/Cone needs radius_eff
    // if (g.geo_type == GeoType::SPHERE || g.geo_type == GeoType::CAPSULE) g.radius_eff = g.geo_scale.x();

    return g;
}









