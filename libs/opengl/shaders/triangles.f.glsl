R"XXX(
#version 330 core

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

uniform mat4 mv_matrix;
in vec3 frag_position, frag_normal;
in vec4 frag_materialColor;

out vec4 color;

const vec3 light_direction = vec3(-0.40825, -0.40825, -0.81650);
const vec4 light_diffuse = vec4(0.8, 0.8, 0.8, 0.0);
const vec4 light_ambient = vec4(0.2, 0.2, 0.2, 1.0);
const vec4 light_specular = vec4(1.0, 1.0, 1.0, 1.0);
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
