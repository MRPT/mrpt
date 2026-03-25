R"XXX(#version 300 es

// VERTEX SHADER: SSAO geometry pre-pass — outputs view-space position and normal
// Jose Luis Blanco Claraco (C) 2026
// Part of the MRPT project

layout(location = 0) in vec3 position;
layout(location = 2) in vec3 vertexNormal;

uniform highp mat4 p_matrix;
uniform highp mat4 v_matrix;
uniform highp mat4 m_matrix;

out highp vec3 frag_viewPos;
out highp vec3 frag_viewNormal;

void main()
{
    highp vec4 worldPos  = m_matrix * vec4(position, 1.0);
    highp vec4 viewPos   = v_matrix * worldPos;
    frag_viewPos         = viewPos.xyz;
    frag_viewNormal      = normalize(mat3(v_matrix) * mat3(m_matrix) * vertexNormal);
    gl_Position          = p_matrix * viewPos;
}
)XXX"
