R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


in vec3 position;
in vec4 vertexColor;

uniform mediump mat4 pmv_matrix;

out mediump vec4 frag_materialColor;

void main()
{
    gl_Position = pmv_matrix * vec4(position, 1.0);
    frag_materialColor = vertexColor;
}
)XXX"
