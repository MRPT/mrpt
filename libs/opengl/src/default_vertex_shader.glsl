R"XXX(
#version 330 core

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019
// Part of the MRPT project


layout(location = 0) in vec3 position;

uniform mat4 p_matrix;
uniform mat4 mv_matrix;

//uniform sampler2D texture;

//attribute vec3 position; //, normal;
//attribute vec4 material_diffuse_color;

//attribute vec2 texcoord;
//attribute float shininess;
//attribute vec4 specular;

//varying vec3 frag_position;
//varying vec3 frag_normal;
//varying vec4 frag_diffuse;

//varying vec2 frag_texcoord;
//varying float frag_shininess;
//varying vec4 frag_specular;

void main()
{
    vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;

/*    frag_position = eye_position.xyz;
    frag_diffuse = material_diffuse_color;
*/
//    frag_normal   = (mv_matrix * vec4(normal, 0.0)).xyz;
//    frag_texcoord = texcoord;
//    frag_shininess = shininess;
//    frag_specular = specular;
}
)XXX"
