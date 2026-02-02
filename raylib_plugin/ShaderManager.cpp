#include "ShaderManager.h"

#include <iostream>
#include <stdexcept>

ShaderManager::ManagedShader &ShaderManager::LoadShaderProgram(const std::string &name, const char *vertexPath, const char *fragmentPath) {
    ManagedShader &managed = EmplaceOrReplace(name);
    managed.shader = LoadShader(vertexPath, fragmentPath);
    managed.viewPosLoc = ShaderManager::CheckSetShaderLocation(managed.shader, "viewPos");
    managed.lightDirLoc = ShaderManager::CheckSetShaderLocation(managed.shader, "lightDir");
    return managed;
}

ShaderManager::ManagedShader *ShaderManager::Get(const std::string &name) {
    const auto it = m_shaders.find(name);
    if (it == m_shaders.end()) {
        throw std::runtime_error("Shader \"" + name + "\" does not exist.");
        return nullptr;
    }
    return &it->second;
}

bool ShaderManager::Has(const std::string &name) const {
    return m_shaders.find(name) != m_shaders.end();
}

void ShaderManager::ApplyShaderToModel(const Model &model, const std::string &name) {
    if (const ManagedShader *shader = Get(name))
    {
        model.materials[0].shader = shader->shader;
    }
}

void ShaderManager::ApplyShaderToModels(const std::vector<Model> &models, const std::string &name) {
    for (auto &model : models)
    {
        ApplyShaderToModel(model, name);
    }
}

void ShaderManager::UpdateViewPos(const std::string &name, const Vector3 &viewPos) {
    const ManagedShader *shader = Get(name);
    if (!shader){
        return;
    }

    if (shader->viewPosLoc >= 0){
        SetShaderValue(shader->shader, shader->viewPosLoc, &viewPos, SHADER_UNIFORM_VEC3);
    }

}

void ShaderManager::UnloadAll() {
    for (auto &entry : m_shaders)
    {
        if (entry.second.shader.id > 0)
        {
            UnloadShader(entry.second.shader);
        }
    }
    m_shaders.clear();
}

ShaderManager::ManagedShader &ShaderManager::EmplaceOrReplace(const std::string &name) {
    // 断点停在这里
    auto s  = m_shaders.size();
    auto bc = m_shaders.bucket_count();
    auto lf = m_shaders.load_factor();

    auto [it, inserted] = m_shaders.emplace(name, ManagedShader{});
    if (!inserted) {
        if (it->second.shader.id > 0) {
            UnloadShader(it->second.shader);
        }
        it->second = ManagedShader{};
    }
    return it->second;
}

void ShaderManager::BindMatrices(const Shader &sh) {
    sh.locs[SHADER_LOC_MATRIX_MVP]    = GetShaderLocation(sh, "mvp");
    sh.locs[SHADER_LOC_MATRIX_MODEL]  = GetShaderLocation(sh, "matModel");
    sh.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(sh, "matNormal");
}

int ShaderManager::CheckSetShaderLocation(const Shader &sh, const char* uniform_val) {
    const int loc_result = GetShaderLocation(sh, uniform_val);
    if (loc_result < 0)
        std::cerr << "[WARN] uniform not found: lightDir\n";
    return loc_result;
}

void ShaderManager::SetCommonShaderParams(const Shader &sh) {
    const int exposure = ShaderManager::CheckSetShaderLocation(sh, "exposure");

    const int skyAmb = ShaderManager::CheckSetShaderLocation(sh, "skyAmbientColor");
    const int gndAmb = ShaderManager::CheckSetShaderLocation(sh, "groundAmbientColor");
    const int ambStr = ShaderManager::CheckSetShaderLocation(sh, "ambientStrength");

    const int lightDir = ShaderManager::CheckSetShaderLocation(sh, "lightDir");
    const int lightColor = ShaderManager::CheckSetShaderLocation(sh, "lightColor");

    const int spotPos = ShaderManager::CheckSetShaderLocation(sh, "spotPos");
    const int spotDir = ShaderManager::CheckSetShaderLocation(sh, "spotDir");
    const int spotColor = ShaderManager::CheckSetShaderLocation(sh, "spotColor");
    const int spotIntensity = ShaderManager::CheckSetShaderLocation(sh, "spotIntensity");
    const int spotRange = ShaderManager::CheckSetShaderLocation(sh, "spotRange");
    const int spotInC = ShaderManager::CheckSetShaderLocation(sh, "spotInnerCos");
    const int spotOuC = ShaderManager::CheckSetShaderLocation(sh, "spotOuterCos");

    const CommonShaderParams params;

    SetShaderValue(sh, exposure, &params.exposure, SHADER_UNIFORM_FLOAT);

    SetShaderValue(sh, skyAmb, &params.skyAmb, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, gndAmb, &params.gndAmb, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, ambStr, &params.ambientStrength, SHADER_UNIFORM_FLOAT);

    SetShaderValue(sh, lightDir, &params.lightDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, lightColor, &params.lightColor, SHADER_ATTRIB_FLOAT);

    SetShaderValue(sh, spotPos, &params.spotPos, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, spotDir, &params.spotDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(sh, spotColor, &params.spotColor, SHADER_ATTRIB_VEC3);
    SetShaderValue(sh, spotIntensity, &params.spotIntensity, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, spotRange, &params.spotRange, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, spotInC, &params.spotInnerCos, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sh, spotOuC, &params.spotOuterCos, SHADER_UNIFORM_FLOAT);
}
