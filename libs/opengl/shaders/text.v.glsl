R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


in vec3 position;
in vec4 vertexColor;

uniform mat4 p_matrix;
uniform mat4 mv_matrix;

out vec4 frag_color;

void main()
{
    highp vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;
    frag_color = vertexColor;
}
)XXX"
