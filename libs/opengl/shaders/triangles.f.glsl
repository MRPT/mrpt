R"XXX(
#version 330 core

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

uniform mat4 mv_matrix;
uniform vec4 light_diffuse, light_ambient, light_specular;
uniform vec3 light_direction;

in vec3 frag_position, frag_normal;
in vec4 frag_materialColor;

out vec4 color;

const float frag_shininess = 1.0;

void main()
{
    vec3 mv_light_direction = (mv_matrix * vec4(light_direction, 0.0)).xyz,
         eye = normalize(frag_position),
         reflection = reflect(mv_light_direction, frag_normal);

    vec4 diffuse_factor = max(-dot(frag_normal, mv_light_direction), 0.0) * light_diffuse;
    vec4 ambient_diffuse_factor = diffuse_factor + light_ambient;
    vec4 specular_factor = max(pow(-dot(reflection, eye), frag_shininess), 0.0) * light_specular;

    color = frag_materialColor * (diffuse_factor + ambient_diffuse_factor + specular_factor);
}
)XXX"
