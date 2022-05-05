R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


in vec3 position;
in vec4 vertexColor;
in vec3 vertexNormal;

uniform highp mat4 p_matrix;
uniform highp mat4 mv_matrix;

out highp vec3 frag_position, frag_normal;
out highp vec4 frag_materialColor;

void main()
{
    mediump vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;

    frag_position = eye_position.xyz;
    frag_materialColor = vertexColor;
    frag_normal   = (mv_matrix * vec4(normalize(vertexNormal), 0.0)).xyz;
}
)XXX"
