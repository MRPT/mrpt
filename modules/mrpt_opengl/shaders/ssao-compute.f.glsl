R"XXX(#version 300 es

// FRAGMENT SHADER: SSAO hemisphere sampling
// Jose Luis Blanco Claraco (C) 2026
// Part of the MRPT project

precision highp float;

in mediump vec2 frag_UV;

out float fragOcclusion;

uniform highp sampler2D gPosition;       // view-space position (RGBA32F, w=1 if valid)
uniform highp sampler2D gNormal;         // view-space normal
uniform mediump sampler2D noiseTexture;  // 4x4 random rotation vectors (RG)

// Up to 64 hemisphere samples in tangent space
uniform highp vec3  ssao_samples[64];
uniform int         ssao_kernel_size;  // actual number of samples used
uniform highp float ssao_radius;
uniform highp float ssao_bias;
// Projection focal lengths in NDC: proj[0][0] and proj[1][1]
uniform highp float proj_fx;
uniform highp float proj_fy;
uniform mediump vec2 noiseScale;       // viewport_size / 4.0

// Project a view-space point to UV [0,1] using only focal lengths.
// View space: camera at origin, -Z forward.
// NDC: x_ndc = proj_fx * x / (-z), y_ndc = proj_fy * y / (-z)
// UV:  u = ndc * 0.5 + 0.5
vec2 viewToUV(highp vec3 pos)
{
    highp float invNegZ = 1.0 / (-pos.z);
    return vec2(proj_fx * pos.x * invNegZ, proj_fy * pos.y * invNegZ) * 0.5 + 0.5;
}

void main()
{
    // Sample G-buffer
    highp vec4 fragPosW  = texture(gPosition, frag_UV);
    highp vec3 fragPos   = fragPosW.xyz;

    // If nothing was rendered here (background), no occlusion
    if (fragPosW.w < 0.5) { fragOcclusion = 1.0; return; }

    // View-space normals point outward from surfaces (standard convention).
    // For front-facing geometry, the view-space normal points toward the camera (+Z).
    highp vec3 normal = normalize(texture(gNormal, frag_UV).xyz);

    // Random rotation vector from tiling noise texture
    mediump vec2 noiseUV  = frag_UV * noiseScale;
    highp vec3 randomVec  = vec3(texture(noiseTexture, noiseUV).rg, 0.0);

    // Build TBN to orient hemisphere along fragment normal
    highp vec3 tangent   = normalize(randomVec - normal * dot(randomVec, normal));
    highp vec3 bitangent = cross(normal, tangent);
    highp mat3 TBN       = mat3(tangent, bitangent, normal);

    // Sample the hemisphere
    highp float occlusion = 0.0;
    for (int i = 0; i < 64; i++)
    {
        if (i >= ssao_kernel_size) break;

        // Transform sample to view space
        highp vec3 samplePos = TBN * ssao_samples[i];
        samplePos = fragPos + samplePos * ssao_radius;

        // Project sample to screen UV using focal lengths
        highp vec2 sampleUV = viewToUV(samplePos);

        // Clamp to valid texture range to avoid wrapping artifacts
        sampleUV = clamp(sampleUV, vec2(0.001), vec2(0.999));

        // Get sample depth from G-buffer
        highp vec4 sampleGBuf = texture(gPosition, sampleUV);
        // If the sample projects to background (no geometry), skip it
        if (sampleGBuf.w < 0.5) continue;
        highp float sampleDepth = sampleGBuf.z;

        // Range check: ignore samples too far from the fragment
        highp float rangeCheck = smoothstep(0.0, 1.0, ssao_radius / (abs(fragPos.z - sampleDepth) + 0.001));
        // Standard OpenGL: -Z forward, so "closer to camera" = larger z.
        // Occluded when surface at projected position is closer to camera
        // (larger/less-negative z) than the sample point.
        occlusion += (sampleDepth >= samplePos.z + ssao_bias ? 1.0 : 0.0) * rangeCheck;
    }
    occlusion = 1.0 - (occlusion / float(ssao_kernel_size));

    fragOcclusion = clamp(occlusion, 0.0, 1.0);
}
)XXX"
