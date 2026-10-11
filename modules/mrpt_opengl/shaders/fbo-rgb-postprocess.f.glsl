R"XXX(#version 300 es

// FRAGMENT SHADER: CFBORender RGB post-processing pass. Warps the image
// rendered with an ideal pinhole camera into a distorted camera image, by
// means of a lookup table, and adds Gaussian noise to each pixel channel.
// Jose Luis Blanco Claraco (C) 2026
// Part of the MRPT project

precision highp float;
precision highp int;

out vec4 fragColor;

// Image rendered by the scene pass (an sRGB texture):
uniform highp sampler2D sceneTex;

// For each output pixel, its normalized coordinates in sceneTex:
uniform highp sampler2D lutTex;
uniform int useLUT;

// Noise standard deviation, in [0,1] intensity units (0: disabled):
uniform float noiseStd;
uniform uint noiseSeed;
uniform uint frameIndex;

// PCG hash (Jarzynski and Olano, "Hash Functions for GPU Rendering", 2020)
uint pcgHash(uint v)
{
    uint state = v * 747796405u + 2891336453u;
    uint word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

// Uniform sample in (0,1), never 0 so its log() is finite:
float uniform01(inout uint state)
{
    state = pcgHash(state);
    return (float(state >> 8u) + 0.5) * (1.0 / 16777216.0);
}

vec3 linearToSRGB(vec3 c)
{
    vec3 lo = c * 12.92;
    vec3 hi = 1.055 * pow(c, vec3(1.0 / 2.4)) - 0.055;
    return mix(hi, lo, vec3(lessThanEqual(c, vec3(0.0031308))));
}

void main()
{
    ivec2 px = ivec2(gl_FragCoord.xy);

    vec3 c;
    if (useLUT != 0)
    {
        c = texture(sceneTex, texelFetch(lutTex, px, 0).xy).rgb;
    }
    else
    {
        c = texelFetch(sceneTex, px, 0).rgb;
    }

    // sRGB textures are decoded to linear values when sampled. Encode them
    // back, so the output keeps the stored 8-bit values:
    c = linearToSRGB(c);

    if (noiseStd > 0.0)
    {
        uint state = pcgHash(uint(px.x) ^ pcgHash(uint(px.y) ^ pcgHash(frameIndex ^ pcgHash(noiseSeed))));

        // Box-Muller transform: 2 uniform samples give 2 Gaussian samples
        float r1 = sqrt(-2.0 * log(uniform01(state)));
        float a1 = 6.28318530718 * uniform01(state);
        float r2 = sqrt(-2.0 * log(uniform01(state)));
        float a2 = 6.28318530718 * uniform01(state);
        c += noiseStd * vec3(r1 * cos(a1), r1 * sin(a1), r2 * cos(a2));
    }

    fragColor = vec4(clamp(c, 0.0, 1.0), 1.0);
}
)XXX"
