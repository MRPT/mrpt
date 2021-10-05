R"XXX(
#version 330 core

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

uniform mat4 mv_matrix;
uniform vec4 light_diffuse, light_ambient;
uniform vec3 light_direction;
uniform int enableLight;  // 0 or 1
uniform sampler2D textureSampler;

in vec3 frag_position, frag_normal;
in vec2 frag_UV; // Interpolated values from the vertex shaders

out vec4 color;

void main()
{
    if (enableLight!=0)
    {
        vec3 mv_light_direction = light_direction;
        vec3 eye = normalize(frag_position);
        vec3 fn = normalize(frag_normal);

        vec4 diffuse_factor = max(-dot(fn, mv_light_direction), 0.0) * light_diffuse;

        color = texture( textureSampler, frag_UV ) * (diffuse_factor + light_ambient);
    }
    else
    {
        color = texture( textureSampler, frag_UV );
    }
}
)XXX"
