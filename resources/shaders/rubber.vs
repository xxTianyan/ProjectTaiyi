#version 330

// Raylib vertex inputs
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;
in vec4 vertexColor;

// Raylib matrices
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;

// Varyings
out vec3 fragPosWS;
out vec3 fragNormalWS;
out vec2 fragUV;
out vec4 fragColor;

void main()
{
    vec4 worldPos = matModel * vec4(vertexPosition, 1.0);
    fragPosWS = worldPos.xyz;

    // matNormal is the inverse-transpose of matModel in raylib convention
    fragNormalWS = normalize((matNormal * vec4(vertexNormal, 0.0)).xyz);

    fragUV = vertexTexCoord;
    fragColor = vertexColor;

    gl_Position = mvp * vec4(vertexPosition, 1.0);
}
