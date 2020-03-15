R"XXX(
#version 330 core

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 1) in vec2 vertexUV;
layout(location = 2) in vec3 vertexNormal;

uniform mat4 p_matrix;
uniform mat4 mv_matrix;
uniform mat4 pmv_matrix;

out vec3 frag_position, frag_normal;
out vec2 frag_UV; // Interpolated UV texture coords
uniform int enableLight;  // 0 or 1

void main()
{
    frag_UV = vertexUV;

    if (enableLight!=0)
    {
        vec4 eye_position = mv_matrix * vec4(position, 1.0);
        gl_Position = p_matrix * eye_position;
        frag_position = eye_position.xyz;
        frag_normal   = normalize((mv_matrix * vec4(vertexNormal, 0.0)).xyz);
    }
    else
    {
        gl_Position = pmv_matrix * vec4(position, 1.0);
    }
}
)XXX"
