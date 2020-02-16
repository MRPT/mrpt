R"XXX(
#version 330 core

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;

uniform mat4 p_matrix;
uniform mat4 mv_matrix;
uniform float vertexPointSize;

out vec4 frag_color;

void main()
{
    vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;
    gl_PointSize = vertexPointSize;

    frag_color = vertexColor;
}
)XXX"
