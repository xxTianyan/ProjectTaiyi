#version 330

in vec3 vWorldPos;
in vec3 vWorldNormal;
in vec2 vTexCoord;
in vec4 vColor;

out vec4 fragColor;

// ====================== [Shared / Common Lighting Params] ======================
// Camera
uniform vec3  viewPos;

// Time (optional for subtle variation; also keeps interface consistent)
uniform float iTime;

// Hemispheric ambient (dark scene friendly)
uniform vec3  skyAmbientColor;      // 上半球环境色（偏冷/偏蓝更像 Newton）
uniform vec3  groundAmbientColor;   // 下半球环境色（偏灰）
uniform float ambientStrength;      // 环境光强度：越小越暗、对比越强

// Weak directional fill light (optional, keep small)
uniform vec3  lightDir;             // 方向光传播方向（Light -> Scene）
uniform vec3  lightColor;           // HDR 可>1，但这里建议很小（补光用）

// Spotlight (the “light pool” / 光圈)
uniform vec3  spotPos;              // 聚光灯位置（World）
uniform vec3  spotDir;              // 聚光灯传播方向（Light -> Scene），需归一化
uniform vec3  spotColor;            // 聚光颜色（线性空间）
uniform float spotIntensity;        // 聚光强度（主要用它来“打出光圈”）
uniform float spotRange;            // 作用距离（越大照得越远）
uniform float spotInnerCos;         // 内锥 cos(θ_in)：中心全亮区域
uniform float spotOuterCos;         // 外锥 cos(θ_out)：软边缘结束（外侧为0）

// Tonemap
uniform float exposure;             // 曝光：越大越亮（配合 HDR）

// ====================== [Material Params - Cloth] ======================
uniform sampler2D texture0;         // Raylib 默认贴图采样器名
uniform vec4 colDiffuse;            // Raylib 材质 albedo tint（material.maps[ALBEDO].color）

uniform float roughness;            // 0..1：越大越“哑光”（布料建议 0.6~0.9）
uniform float specStrength;         // 高光强度（布料建议 0.10~0.35）
uniform float wrapDiffuse;          // 0..0.5：漫反射“包裹”程度（布料建议 0.15~0.35）

// ====================== [Debug / Two-Sided Tint] ======================
// 正面/背面颜色tint：用于观察自碰撞时正反面翻转与穿插
uniform vec3  frontTint;            // e.g. vec3(0.80, 0.85, 0.95)
uniform vec3  backTint;             // e.g. vec3(0.95, 0.25, 0.25)
uniform float twoSidedTintStrength; // 0..1, 0=off, 1=full tint

// ------------------ Helpers ------------------
vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

// Soft spotlight factor (cone + range attenuation)
float spotAttenuation(vec3 worldPos)
{
    vec3 toP = worldPos - spotPos;               // light -> point
    float dist = length(toP);
    vec3 Sd = normalize(spotDir);                // light direction (light->scene)

    // angular falloff
    float cosAngle = dot(normalize(toP), Sd);    // 1 at center, smaller at edges
    float cone = smoothstep(spotOuterCos, spotInnerCos, cosAngle);

    // range falloff (smooth)
    float rangeAtt = 1.0 - clamp(dist / max(spotRange, 1e-6), 0.0, 1.0);
    rangeAtt = rangeAtt * rangeAtt;

    return cone * rangeAtt;
}

void main()
{
    // Double-sided normal (cloth often needs this)
    vec3 N = normalize(gl_FrontFacing ? vWorldNormal : -vWorldNormal);
    vec3 V = normalize(viewPos - vWorldPos);

    // Base albedo (you can ignore texture0 if you want solid color)
    vec4 texel = texture(texture0, vTexCoord);
    vec4 base  = colDiffuse * texel * vColor;
    vec3 albedo = base.rgb;

    // ------------------ Two-sided tint (debug aid) ------------------
    // 仅调制 albedo，不直接替换光照结果，避免破坏明暗关系
    float tintW = clamp(twoSidedTintStrength, 0.0, 1.0);
    vec3 tint = gl_FrontFacing ? frontTint : backTint;

    // 乘法 tint 更适合“保持纹理/明暗”，比直接 replace 更自然
    vec3 albedoTinted = albedo * tint;

    // 若你希望更“纯色对比”，可把乘法换成 mix(albedo, tint, tintW)
    albedo = mix(albedo, albedoTinted, tintW);

    // ------------------ Ambient (Hemispheric) ------------------
    float hemi = clamp(N.y * 0.5 + 0.5, 0.0, 1.0);
    vec3 ambientHemi = mix(groundAmbientColor, skyAmbientColor, hemi);
    vec3 ambient = ambientStrength * ambientHemi * albedo;

    // ------------------ Weak directional fill (optional) ------------------
    // Directional ray direction: lightDir (light->scene), so surface->light is Ld = -lightDir
    vec3 Ld = normalize(-lightDir);
    float NdotLd = dot(N, Ld);

    float diffD;
    if (wrapDiffuse > 0.0)
        diffD = clamp((NdotLd + wrapDiffuse) / (1.0 + wrapDiffuse), 0.0, 1.0);
    else
        diffD = max(NdotLd, 0.0);

    vec3 dirDiffuse = albedo * diffD * lightColor;

    // Directional spec (keep subtle)
    vec3 Hd = normalize(Ld + V);
    float shininess = mix(256.0, 16.0, clamp(roughness, 0.0, 1.0)); // rough high -> broad highlight
    float specPowD = pow(max(dot(N, Hd), 0.0), shininess);

    vec3 F0 = vec3(0.04);
    vec3 Fd = fresnelSchlick(max(dot(V, Hd), 0.0), F0);
    vec3 dirSpec = specStrength * 0.35 * specPowD * Fd * lightColor * max(NdotLd, 0.0); // 0.35: reduce dir spec

    // ------------------ Spotlight (main character) ------------------
    // For spotlight, the surface->light direction depends on position
    vec3 toP = vWorldPos - spotPos;           // light -> point
    float dist = length(toP);
    vec3 Ls = normalize(-toP);                // point -> light

    float spotAtt = spotAttenuation(vWorldPos);
    float NdotLs = max(dot(N, Ls), 0.0);

    // Spotlight diffuse
    float diffS;
    if (wrapDiffuse > 0.0)
        diffS = clamp((dot(N, Ls) + wrapDiffuse) / (1.0 + wrapDiffuse), 0.0, 1.0);
    else
        diffS = NdotLs;

    vec3 spotDiffuse = albedo * diffS * spotColor * (spotIntensity * spotAtt);

    // Spotlight spec (makes “light pool” feel more real)
    vec3 Hs = normalize(Ls + V);
    float specPowS = pow(max(dot(N, Hs), 0.0), shininess);
    vec3 Fs = fresnelSchlick(max(dot(V, Hs), 0.0), F0);
    vec3 spotSpec = specStrength * specPowS * Fs * spotColor * (spotIntensity * spotAtt) * NdotLs;

    // ------------------ Combine in linear HDR ------------------
    vec3 color = ambient + dirDiffuse + dirSpec + spotDiffuse + spotSpec;

    // ------------------ Tonemap + Gamma ------------------
    vec3 mapped = vec3(1.0) - exp(-color * max(exposure, 0.0001));
    mapped = pow(mapped, vec3(1.0/2.2));

    fragColor = vec4(mapped, base.a);
}
