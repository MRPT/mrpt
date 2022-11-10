R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

uniform mediump vec4 light_diffuse, light_ambient;
uniform mediump vec3 light_direction;

in mediump vec3 frag_position, frag_normal;
in mediump vec4 frag_materialColor;

out mediump vec4 color;

void main()
{
    mediump float diff = max(dot(normalize(frag_normal), -light_direction), 0.0);
    mediump vec4 diffuse_factor = diff * light_diffuse;

    color = frag_materialColor * (diffuse_factor + light_ambient);
}
)XXX"
