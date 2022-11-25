R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2022
// Part of the MRPT project

uniform mediump sampler2D textureSampler;

in mediump vec2 frag_UV; // Interpolated values from the vertex shaders

out mediump vec4 color;

void main()
{
    color = texture( textureSampler, frag_UV );
}

)XXX"
