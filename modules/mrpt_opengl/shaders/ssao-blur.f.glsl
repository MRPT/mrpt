R"XXX(#version 300 es

// FRAGMENT SHADER: 4x4 box blur on the raw SSAO texture
// Jose Luis Blanco Claraco (C) 2026
// Part of the MRPT project

precision mediump float;

in mediump vec2 frag_UV;

out float fragBlurred;

uniform mediump sampler2D ssaoInput;

void main()
{
    mediump vec2 texelSize = 1.0 / vec2(textureSize(ssaoInput, 0));
    float result = 0.0;
    for (int x = -2; x <= 2; ++x)
    {
        for (int y = -2; y <= 2; ++y)
        {
            mediump vec2 offset = vec2(float(x), float(y)) * texelSize;
            result += texture(ssaoInput, frag_UV + offset).r;
        }
    }
    fragBlurred = result / 25.0;
}
)XXX"
