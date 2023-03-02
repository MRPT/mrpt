R"XXX(#version 300 es
#extension GL_OES_shader_io_blocks: require

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project


in vec3 position;
in vec2 vertexUV;
in vec3 vertexNormal;

uniform highp mat4 p_matrix;
uniform highp mat4 v_matrix;
uniform highp mat4 m_matrix;
uniform highp mat4 light_pv_matrix; // =p*v matrices

out Fragment {
    highp vec3 position, normal;
    out mediump vec2 UV; // Interpolated UV texture coords
    highp vec4 posLightSpace;
} frag;

void main()
{
    highp vec4 vPos    = m_matrix * vec4(position, 1.0);
    frag.position      = vec3(vPos);
    frag.normal        = normalize(mat3(m_matrix) * vertexNormal);
    frag.UV            = vertexUV;

    frag.posLightSpace = light_pv_matrix     * vPos;
    gl_Position        = p_matrix * v_matrix * vPos;
}
)XXX"
