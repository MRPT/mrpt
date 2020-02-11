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

//uniform sampler2D texture;

//attribute vec3 position;
//attribute vec4 material_diffuse_color;

//attribute vec2 texcoord;
//attribute float shininess;
//attribute vec4 specular;

out vec3 frag_position, frag_normal;
out vec4 frag_diffuse;

//varying vec2 frag_texcoord;
//varying float frag_shininess;
//varying vec4 frag_specular;

void main()
{
    vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;
    //gl_PointSize = 4.0;

    // The color of each vertex will be interpolated
    // to produce the color of each fragment
    frag_position = eye_position.xyz;
    frag_diffuse = vertexColor;
    frag_normal   = normalize((mv_matrix * vec4(vertexNormal, 0.0)).xyz);
//    frag_texcoord = texcoord;
//    frag_shininess = shininess;
//    frag_specular = specular;
}
)XXX"
