R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

uniform mediump sampler2D textureSampler;

in mediump vec2 frag_UV; // Interpolated values from the vertex shaders
in lowp vec4 frag_vertexColor;

out lowp vec4 color;

void main()
{
    lowp vec4 texColor = texture(textureSampler, frag_UV);
    color = texColor * frag_vertexColor;
}

)XXX"
