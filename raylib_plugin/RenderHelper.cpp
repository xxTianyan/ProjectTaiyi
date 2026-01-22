//
// Created by tianyan on 1/6/26.
//

#include "RenderHelper.h"


void RenderHelper::BindModel(const MModel& model) {
    model_ = &model;
}

void RenderHelper::Shutdown() {
    for (auto& rm : meshes_) {
        if (rm.valid) {
            UnloadRLModelSafe(rm.model);
            rm.valid = false;
        }
    }
    meshes_.clear();
    ready_ = false;
    built_topology_version_ = 0;
}

void RenderHelper::Update(const State& state) {
    if (model_ == nullptr) return;

    // 1) consistency check
    if (model_->num_particles != state.particle_pos.size()) {
        throw std::runtime_error("RenderHelper::Update: model.num_particles != state.particle_pos.size()");
    }

    // 2) rebuild if topology changed
    if (!ready_ || built_topology_version_ != model_->topology_version) {
        Rebuild();
        built_topology_version_ = model_->topology_version;
        ready_ = true;
    }

    // 3) update positions + normals
    UpdateDynamic(state);
}

bool RenderHelper::IsModelValid(const Model &rl_model) {
    return (rl_model.meshCount > 0 && rl_model.meshes != nullptr);
}

void RenderHelper::UnloadRLModelSafe(Model &rl_model) {
    if (IsModelValid(rl_model)) {
        UnloadModel(rl_model);
        rl_model = Model{}; // make raylib model invalid
    }
}

Model& RenderHelper::GetRLModel(const size_t mesh_id) {
    return meshes_[mesh_id].model;
}

void RenderHelper::Rebuild() {
    if (model_ == nullptr) return;

    // safe release
    Shutdown();

    meshes_.reserve(model_->mesh_infos.size());

    for (const MeshInfo& info_src : model_->mesh_infos) {
        RenderMesh rm{};
        rm.info = info_src;

        const size_t particle_begin = rm.info.particle.begin;
        const size_t particle_count = rm.info.particle.count;
        const size_t tri_count      = rm.info.render_tri.count;

        // important
        if (particle_count == 0 || tri_count == 0) {
            throw std::runtime_error("RenderHelper::Rebuild: particle_count or tri_count is 0");
            rm.valid = false;
            meshes_.push_back(rm);
            continue;
        }

        // raylib Mesh indices is u16：the number of a single mesh must be less than 0xFFFF
        if (particle_count > static_cast<size_t>(std::numeric_limits<unsigned short>::max()) + 1ull) {
            throw std::runtime_error("RenderHelper::Rebuild: particle_count > 65536, raylib u16 indices not supported. Split mesh.");
        }

        // check valid range
        if (particle_begin + particle_count > model_->num_particles) {
            throw std::runtime_error("RenderHelper::Rebuild: particle range out of model.num_particles");
        }
        if (rm.info.render_tri.end() > model_->render_tris.size()) {
            throw std::runtime_error("RenderHelper::Rebuild: tri range out of model.render_tris");
        }

        // -------- build CPU mesh arrays (MemAlloc, owned by raylib) --------
        Mesh mesh{};
        mesh.vertexCount   = static_cast<int>(particle_count);
        mesh.triangleCount = static_cast<int>(tri_count);

        mesh.vertices = static_cast<float *>(MemAlloc(particle_count * 3 * sizeof(float)));
        mesh.normals  = static_cast<float *>(MemAlloc(particle_count * 3 * sizeof(float)));
        mesh.indices  = static_cast<unsigned short *>(MemAlloc(tri_count * 3 * sizeof(unsigned short)));  // triangle index

        if (!mesh.vertices || !mesh.normals || !mesh.indices) {
            throw std::runtime_error("RenderHelper::Rebuild: MemAlloc failed");
        }

        // 这里用 particle_pos0 初始化也可以；但更稳妥做法是：首次重建时也由 Update(state) 负责写
        // 为避免多做一次拷贝，我们先填充一个合理初值（零），随后 UpdateDynamic 会覆盖
        std::memset(mesh.vertices, 0, particle_count * 3 * sizeof(float));
        std::memset(mesh.normals,  0, particle_count * 3 * sizeof(float));

        BuildIndicesU16(*model_, rm.info.render_tri, particle_begin, particle_count, (unsigned short*)mesh.indices);

        // Upload as dynamic: positions/normals 每帧更新
        UploadMesh(&mesh, true);

        rm.model = LoadModelFromMesh(mesh);
        rm.model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
        if (IsModelValid(rm.model)) rm.valid = true;
        meshes_.push_back(rm);
    }

    ready_ = true;
}

void RenderHelper::UpdateDynamic(const State& state) const {
    if (model_ == nullptr) return;
    if (!ready_) return;

    for (auto& rm : meshes_) {
        if (!rm.valid) continue;

        const size_t particle_begin = rm.info.particle.begin;
        const size_t particle_count = rm.info.particle.count;

        const Mesh& mesh = rm.model.meshes[0];

        if (static_cast<size_t>(mesh.vertexCount) != particle_count) {
            throw std::runtime_error("RenderHelper::UpdateDynamic: mesh.vertexCount mismatch (topology out of sync)");
        }
        if (!mesh.vertices || !mesh.normals) {
            throw std::runtime_error("RenderHelper::UpdateDynamic: mesh CPU arrays are null");
        }

        FillPositionsXYZ(state, particle_begin, particle_count, mesh.vertices);
        ComputeNormalsXYZ(*model_, state, rm.info.render_tri, particle_begin, particle_count, mesh.normals);

        // raylib buffer slots: 0=positions, 2=normals
        UpdateMeshBuffer(mesh, 0, mesh.vertices, mesh.vertexCount * 3 * static_cast<int>(sizeof(float)), 0);
        UpdateMeshBuffer(mesh, 2, mesh.normals,  mesh.vertexCount * 3 * static_cast<int>(sizeof(float)), 0);
    }
}

void RenderHelper::Draw(const bool is_wire_mode) const {
    if (!ready_) return;

    for (const auto& rm : meshes_) {
        if (!rm.valid) continue;
        if (is_wire_mode)
            DrawModelWires(rm.model, Vector3{0.0f, 0.0f, 0.0f}, 1.0, BLACK);
        else
            DrawModel(rm.model, Vector3{0.0f, 0.0f, 0.0f}, 1.0f, WHITE);
    }
}

// ------------------------ math helpers ------------------------

void RenderHelper::FillPositionsXYZ(const State& state,
                                    const size_t particle_begin,
                                    const size_t particle_count,
                                    float* dst_xyz) {
    if (!dst_xyz) throw std::runtime_error("FillPositionsXYZ: dst_xyz null");
    if (particle_begin + particle_count > state.particle_pos.size()) {
        throw std::runtime_error("FillPositionsXYZ: particle range out of state.particle_pos");
    }

    for (size_t i = 0; i < particle_count; ++i) {
        const Vec3& p = state.particle_pos[particle_begin + i];
        dst_xyz[i * 3 + 0] = p.x();
        dst_xyz[i * 3 + 1] = p.y();
        dst_xyz[i * 3 + 2] = p.z();
    }
}

void RenderHelper::BuildIndicesU16(const MModel& model,
                                  const range tri_range,
                                  const size_t particle_begin,
                                  const size_t particle_count,
                                  unsigned short* dst_indices) {
    if (!dst_indices) throw std::runtime_error("BuildIndicesU16: dst null");
    if (tri_range.end() > model.render_tris.size()) {
        throw std::runtime_error("BuildIndicesU16: tri range out of model.render_tris");
    }

    for (size_t t = 0; t < tri_range.count; ++t) {
        const auto& tri = model.render_tris[tri_range.begin + t];

        const VertexID g0 = tri.vertices[0];
        const VertexID g1 = tri.vertices[1];
        const VertexID g2 = tri.vertices[2];

        if (g0 < particle_begin || g1 < particle_begin || g2 < particle_begin) {
            throw std::runtime_error("BuildIndicesU16: triangle references vertex before particle_begin");
        }

        const auto i0 = g0 - particle_begin;
        const auto i1 = g1 - particle_begin;
        const auto i2 = g2 - particle_begin;

        if (i0 >= particle_count || i1 >= particle_count || i2 >= particle_count) {
            throw std::runtime_error("BuildIndicesU16: triangle references vertex out of this mesh particle range");
        }

        dst_indices[t * 3 + 0] = static_cast<unsigned short>(i0);
        dst_indices[t * 3 + 1] = static_cast<unsigned short>(i1);
        dst_indices[t * 3 + 2] = static_cast<unsigned short>(i2);
    }
}

void RenderHelper::ComputeNormalsXYZ(const MModel& model,
                                     const State& state,
                                     const range tri_range,
                                     const size_t particle_begin,
                                     const size_t particle_count,
                                     float* dst_nxyz) {
    if (!dst_nxyz) throw std::runtime_error("ComputeNormalsXYZ: dst null");
    if (particle_begin + particle_count > state.particle_pos.size()) {
        throw std::runtime_error("ComputeNormalsXYZ: particle range out of state.particle_pos");
    }
    if (tri_range.end() > model.render_tris.size()) {
        throw std::runtime_error("ComputeNormalsXYZ: tri range out of model.render_tris");
    }

    std::memset(dst_nxyz, 0, particle_count * 3 * sizeof(float));

    for (size_t t = 0; t < tri_range.count; ++t) {
        const auto& tri = model.render_tris[tri_range.begin + t];

        const VertexID g0 = tri.vertices[0];
        const VertexID g1 = tri.vertices[1];
        const VertexID g2 = tri.vertices[2];

        // 映射到 mesh 局部 index
        if (g0 < particle_begin || g1 < particle_begin || g2 < particle_begin) continue;

        const auto i0 = static_cast<size_t>(g0 - particle_begin);
        const auto i1 = static_cast<size_t>(g1 - particle_begin);
        const auto i2 = static_cast<size_t>(g2 - particle_begin);

        if (i0 >= particle_count || i1 >= particle_count || i2 >= particle_count) continue;

        const Vec3& p0 = state.particle_pos[particle_begin + i0];
        const Vec3& p1 = state.particle_pos[particle_begin + i1];
        const Vec3& p2 = state.particle_pos[particle_begin + i2];

        const Vec3 n = (p1 - p0).cross(p2 - p0); // area-weighted
        const float nx = n.x(), ny = n.y(), nz = n.z();

        dst_nxyz[i0 * 3 + 0] += nx; dst_nxyz[i0 * 3 + 1] += ny; dst_nxyz[i0 * 3 + 2] += nz;
        dst_nxyz[i1 * 3 + 0] += nx; dst_nxyz[i1 * 3 + 1] += ny; dst_nxyz[i1 * 3 + 2] += nz;
        dst_nxyz[i2 * 3 + 0] += nx; dst_nxyz[i2 * 3 + 1] += ny; dst_nxyz[i2 * 3 + 2] += nz;
    }

    // normalize
    for (size_t i = 0; i < particle_count; ++i) {
        constexpr float eps = 1e-12f;
        const float x = dst_nxyz[i * 3 + 0];
        const float y = dst_nxyz[i * 3 + 1];
        const float z = dst_nxyz[i * 3 + 2];

        if (const float len2 = x * x + y * y + z * z; len2 > eps) {
            const float inv = 1.0f / std::sqrt(len2);
            dst_nxyz[i * 3 + 0] = x * inv;
            dst_nxyz[i * 3 + 1] = y * inv;
            dst_nxyz[i * 3 + 2] = z * inv;
        } else {
            // fallback
            dst_nxyz[i * 3 + 0] = 0.0f;
            dst_nxyz[i * 3 + 1] = 1.0f;
            dst_nxyz[i * 3 + 2] = 0.0f;
        }
    }
}

