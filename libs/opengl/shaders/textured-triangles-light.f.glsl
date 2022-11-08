R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

uniform mediump mat4 mv_matrix;
uniform mediump vec4 light_diffuse, light_ambient;
uniform mediump vec3 light_direction;
uniform mediump sampler2D textureSampler;

in mediump vec3 frag_position, frag_normal;
in mediump vec2 frag_UV; // Interpolated values from the vertex shaders

out mediump vec4 color;

void main()
{
    mediump vec3 mv_light_direction = light_direction;
    mediump vec3 eye = normalize(frag_position);
    mediump vec3 fn = normalize(frag_normal);
    
    mediump vec4 diffuse_factor = max(-dot(fn, mv_light_direction), 0.0) * light_diffuse;
    
    color = texture( textureSampler, frag_UV ) * (diffuse_factor + light_ambient);
}

)XXX"
