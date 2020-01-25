R"XXX(
#version 330 core

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019
// Part of the MRPT project

//uniform mat4 p_matrix, mv_matrix;
//uniform sampler2D texture;

out vec4 color;

// Interpolated values from the vertex shaders
in vec4 fragmentColor;

//varying vec3 frag_position, frag_normal;
//varying vec2 frag_texcoord;
//varying vec4 frag_diffuse;

/*
varying vec2 frag_texcoord;
varying float frag_shininess;
varying vec4 frag_specular;

const vec3 light_direction = vec3(0.408248, -0.816497, 0.408248);
const vec4 light_diffuse = vec4(0.8, 0.8, 0.8, 0.0);
const vec4 light_ambient = vec4(0.2, 0.2, 0.2, 1.0);
const vec4 light_specular = vec4(1.0, 1.0, 1.0, 1.0);
*/

void main()
{
    // Output color = color specified in the vertex shader,
    // interpolated between all 3 surrounding vertices
    color = fragmentColor;

/*    vec3 mv_light_direction = (mv_matrix * vec4(light_direction, 0.0)).xyz,
         normal = normalize(frag_normal),
         eye = normalize(frag_position),
         reflection = reflect(mv_light_direction, normal);

    //vec4 frag_diffuse = texture2D(texture, frag_texcoord);
    vec4 diffuse_factor
        = max(-dot(normal, mv_light_direction), 0.0) * light_diffuse;
    vec4 ambient_diffuse_factor
        = diffuse_factor + light_ambient;
    vec4 specular_factor
        = max(pow(-dot(reflection, eye), frag_shininess), 0.0) * light_specular;
    vec4 frag_diffuse 

    gl_FragColor = ambient_diffuse_factor * frag_diffuse;
    //+specular_factor * frag_specular 
*/
}
)XXX"
