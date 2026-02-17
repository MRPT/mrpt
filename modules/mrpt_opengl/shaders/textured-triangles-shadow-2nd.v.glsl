R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 2) in vec3 vertexNormal;
layout(location = 3) in vec2 vertexUV;

uniform highp mat4 p_matrix;
uniform highp mat4 v_matrix;
uniform highp mat4 m_matrix;
uniform highp mat4 light_pv_matrix; // =p*v matrices

out highp vec3 frag_position, frag_normal;
out mediump vec2 frag_UV; // Interpolated UV texture coords
out highp vec4 frag_posLightSpace;

void main()
{
    highp vec4 vPos    = m_matrix * vec4(position, 1.0);
    frag_position      = vec3(vPos);
    frag_normal        = normalize(mat3(m_matrix) * vertexNormal);
    frag_UV            = vertexUV;

    frag_posLightSpace = light_pv_matrix     * vPos;
    gl_Position        = p_matrix * v_matrix * vPos;
}
)XXX"
