R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project


in vec3 position;
in vec2 vertexUV;
in vec3 vertexNormal;

uniform mediump mat4 p_matrix;
uniform mediump mat4 v_matrix;
uniform mediump mat4 m_matrix;

out mediump vec3 frag_position, frag_normal;
out mediump vec2 frag_UV; // Interpolated UV texture coords

void main()
{
    frag_UV = vertexUV;

    frag_position = vec3(m_matrix * vec4(position, 1.0));
    frag_normal   = normalize(mat3(m_matrix) * vertexNormal);

    gl_Position = p_matrix * v_matrix * vec4(frag_position, 1.0);
}

)XXX"
