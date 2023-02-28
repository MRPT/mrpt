R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

in vec3 position;
in vec4 vertexColor;

uniform highp mat4 m_matrix;
uniform highp mat4 light_matrix;

void main()
{
    gl_Position = light_matrix * m_matrix * vec4(position, 1.0);
}
)XXX"
