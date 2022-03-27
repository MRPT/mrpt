R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

uniform highp mat4 mv_matrix;
uniform highp vec4 light_diffuse, light_ambient;
uniform highp vec3 light_direction;
uniform lowp int enableLight;  // 0 or 1

in highp vec3 frag_position, frag_normal;
in highp vec4 frag_materialColor;

out highp vec4 color;

void main()
{
    if (enableLight!=0)
    {
        highp vec3 mv_light_direction = light_direction;
        highp vec3 eye = normalize(frag_position);
        highp vec3 fn = normalize(frag_normal);

        highp vec4 diffuse_factor = max(-dot(frag_normal, mv_light_direction), 0.0) * light_diffuse;

        color = frag_materialColor * (diffuse_factor + light_ambient);
    }
    else
    {
        color = frag_materialColor;
    }
}
)XXX"
