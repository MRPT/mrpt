R"XXX(
#version 330 core

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal;

uniform mat4 p_matrix;
uniform mat4 mv_matrix;

out vec3 frag_position, frag_normal;
out vec4 frag_diffuse;

void main()
{
    vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;

    frag_position = eye_position.xyz;
    frag_diffuse = vertexColor;
    frag_normal   = normalize((mv_matrix * vec4(vertexNormal, 0.0)).xyz);
}
)XXX"
