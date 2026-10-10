R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project

layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal;
layout(location = 3) in vec2 vertexUV;
layout(location = 4) in vec4 vertexTangent;  // w: UV mapping handedness (+1 or -1)
layout(location = 5) in highp mat4 instanceMatrix;  // identity unless instanced (INSTANCE_MATRIX_ATTRIB_LOCATION)

uniform highp mat4 p_matrix;
uniform highp mat4 v_matrix;
uniform highp mat4 m_matrix;
uniform highp mat3 normal_matrix;  // inverse transpose of mat3(m_matrix)

out highp vec3 frag_position, frag_normal;
out mediump vec2 frag_UV; // Interpolated UV texture coords
out lowp vec4 frag_vertexColor;
out highp vec4 frag_tangent;

void main()
{
    highp mat3 tangentMatrix = mat3(m_matrix) * mat3(instanceMatrix);
    highp vec4 vPos    = m_matrix * (instanceMatrix * vec4(position, 1.0));
    frag_position      = vec3(vPos);
    frag_normal        = normalize(normal_matrix * (mat3(instanceMatrix) * vertexNormal));
    frag_UV            = vertexUV;
    frag_vertexColor   = vertexColor;

    // Transform tangent to world space
    frag_tangent = vec4(normalize(tangentMatrix * vertexTangent.xyz), vertexTangent.w);

    gl_Position        = p_matrix * v_matrix * vPos;
}
)XXX"
