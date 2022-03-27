R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal;

uniform mediump mat4 p_matrix;
uniform mediump mat4 mv_matrix;

out mediump vec3 frag_position, frag_normal;
out mediump vec4 frag_materialColor;

void main()
{
    mediump vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;

    frag_position = eye_position.xyz;
    frag_materialColor = vertexColor;
    frag_normal   = (mv_matrix * vec4(normalize(vertexNormal), 0.0)).xyz;
}
)XXX"
