R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project


in vec3 position;
in vec4 vertexColor;
in vec3 vertexNormal;

uniform highp mat4 p_matrix;
uniform highp mat4 v_matrix;
uniform highp mat4 m_matrix;
uniform highp mat4 light_pv_matrix; // =p*v matrices

out Fragment {
    highp vec3 position, normal;
    lowp vec4 materialColor;
    highp vec4 posLightSpace;
} frag;

void main()
{
    highp vec4 vPos    = m_matrix * vec4(position, 1.0);
    frag.position      = vec3(vPos);
    frag.normal        = normalize(mat3(m_matrix) * vertexNormal);
    frag.materialColor = vertexColor;

    frag.posLightSpace = light_pv_matrix     * vPos;
    gl_Position        = p_matrix * v_matrix * vPos;
}
)XXX"
