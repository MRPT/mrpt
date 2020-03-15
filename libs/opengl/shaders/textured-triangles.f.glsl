R"XXX(
#version 330 core

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

uniform mat4 mv_matrix;
uniform vec4 light_diffuse, light_ambient, light_specular;
uniform vec3 light_direction;
uniform sampler2D textureSampler;
uniform int enableLight;  // 0 or 1

in vec3 frag_position, frag_normal;
in vec2 frag_UV; // Interpolated values from the vertex shaders

out vec4 color;

const float frag_shininess = 1.0;

void main()
{
    if (enableLight!=0)
    {
        vec3 mv_light_direction = (mv_matrix * vec4(light_direction, 0.0)).xyz;
        vec3 eye = normalize(frag_position);
        vec3 reflection = reflect(mv_light_direction, frag_normal);
        vec4 diffuse_factor = max(-dot(frag_normal, mv_light_direction), 0.0) * light_diffuse;
        vec4 ambient_diffuse_factor = diffuse_factor + light_ambient;
        vec4 specular_factor = max(pow(-dot(reflection, eye), frag_shininess), 0.0) * light_specular;

        color = texture( textureSampler, frag_UV ) * (diffuse_factor + ambient_diffuse_factor + specular_factor);
    }
    else
    {
        color = texture( textureSampler, frag_UV );
    }
}
)XXX"
