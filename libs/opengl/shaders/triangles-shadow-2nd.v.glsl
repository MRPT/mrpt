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

out Fragment {
    highp vec3 position, normal;
    lowp vec4 materialColor;
} frag;

void main()
{
    frag.position = vec3(m_matrix * vec4(position, 1.0));
    frag.normal   = normalize(mat3(m_matrix) * vertexNormal);
    frag.materialColor = vertexColor;

    gl_Position = p_matrix * v_matrix * vec4(frag.position, 1.0);
}
)XXX"
