R"XXX(#version 300 es

// "#include" file for shadow calculation in fragment shaders (2nd pass)
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project

// Multi-light support (up to 8 lights)
#define MAX_LIGHTS 8
#define MAX_SHADOW_CASCADES 4

uniform int num_lights;
uniform int light_type[MAX_LIGHTS];       // 0=directional, 1=point, 2=spot
uniform lowp vec3 light_color[MAX_LIGHTS];
uniform mediump float light_diffuse[MAX_LIGHTS];
uniform mediump float light_specular[MAX_LIGHTS];
uniform highp vec3 light_direction[MAX_LIGHTS];
uniform highp vec3 light_position[MAX_LIGHTS];
uniform highp vec3 light_attenuation[MAX_LIGHTS]; // (constant, linear, quadratic)
uniform mediump vec2 light_spot_cutoff[MAX_LIGHTS]; // (cos_inner, cos_outer)

uniform mediump float light_ambient;
uniform lowp vec3 ambient_sky_color;
uniform lowp vec3 ambient_ground_color;

uniform bool fog_enabled;
uniform lowp vec3 fog_color;
uniform highp float fog_near;
uniform highp float fog_far;
uniform int fog_mode;
uniform highp float fog_density;

// Cascaded shadow map (hardware depth comparison + bilinear filtering)
uniform highp sampler2DArrayShadow shadowMapArray;
uniform int num_shadow_cascades;
uniform highp mat4 cascade_light_pv[MAX_SHADOW_CASCADES];
uniform highp float cascade_far_planes[MAX_SHADOW_CASCADES];

uniform highp float shadow_bias, shadow_bias_cam2frag, shadow_bias_normal;

// v_matrix is uploaded per-shader in processRenderQueue
uniform highp mat4 v_matrix;

mediump float ShadowCalculation(
    highp vec3 fragWorldPos,
    mediump vec3 normal,
    mediump float cam2fragDist)
{
    // Compute view-space depth for cascade selection
    highp float viewDepth = abs((v_matrix * vec4(fragWorldPos, 1.0)).z);

    // Select cascade: find the first cascade whose far plane covers this fragment
    int cascade = num_shadow_cascades - 1;
    for (int i = 0; i < MAX_SHADOW_CASCADES; i++) {
        if (i >= num_shadow_cascades) break;
        if (viewDepth < cascade_far_planes[i]) {
            cascade = i;
            break;
        }
    }

    // Transform fragment to the selected cascade's light space
    highp vec4 fragPosLightSpace = cascade_light_pv[cascade] * vec4(fragWorldPos, 1.0);

    // Perspective divide
    highp vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;

    // Transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;

    // No shadow if outside the cascade's shadow map coverage
    if (projCoords.z > 1.0 ||
        projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0)
        return 0.0;

    highp float currentDepth = projCoords.z;

    // Shadow bias: normal-dependent term uses NdotL (dot with direction-toward-light = -light_direction).
    // At grazing angles NdotL->0 so bias grows to prevent shadow acne on slanted surfaces.
    highp float bias = shadow_bias + shadow_bias_cam2frag * cam2fragDist +
                       shadow_bias_normal * (1.0 - max(0.0, dot(normal, -light_direction[0])));

    // PCF 3x3 with hardware bilinear shadow comparison.
    // Each texture() call does 2x2 bilinear-filtered depth comparison,
    // so 3x3 samples effectively cover a 4x4 texel area (36 comparisons)
    // giving smooth shadow edges without bloating thin shadow casters.
    highp float refDepth = currentDepth - bias;
    mediump float shadow = 0.0;
    mediump vec2 texelSize = 1.0 / vec2(textureSize(shadowMapArray, 0).xy);
    for (int x = -1; x <= 1; ++x)
    {
        for (int y = -1; y <= 1; ++y)
        {
            shadow += 1.0 - texture(
                shadowMapArray,
                vec4(projCoords.xy + vec2(x, y) * texelSize, float(cascade), refDepth)
            );
        }
    }
    shadow /= 9.0;
    return shadow;
}
)XXX"
