R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

uniform mediump sampler2D textureSampler;
uniform mediump float alphaCutoff; // >0: cutout, <0: opaque, 0: blend

in mediump vec2 frag_UV; // Interpolated values from the vertex shaders
in lowp vec4 frag_vertexColor;

out lowp vec4 color;

void main()
{
    lowp vec4 texCol = texture(textureSampler, frag_UV) * frag_vertexColor;
    // alphaCutoff > 0: cutout (discard, and keep the rest opaque);
    // < 0: opaque; 0: alpha blending.
    if (texCol.a < alphaCutoff)
    {
        discard;
    }
    if (alphaCutoff != 0.0)
    {
        texCol.a = 1.0;
    }
    color = texCol;
}

)XXX"
