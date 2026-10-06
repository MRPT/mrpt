R"XXX(#version 300 es

// "#include" file for shadow calculation in fragment shaders (2nd pass)
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project

// Multi-light support (up to 8 lights)
#define MAX_LIGHTS 8
#define MAX_SHADOW_CASCADES 4
#define MAX_SHADOW_POINT_LIGHTS 8

uniform int num_lights;
uniform int light_type[MAX_LIGHTS];       // 0=directional, 1=point, 2=spot
uniform lowp vec3 light_color[MAX_LIGHTS];
uniform mediump float light_diffuse[MAX_LIGHTS];
uniform mediump float light_specular[MAX_LIGHTS];
uniform highp vec3 light_direction[MAX_LIGHTS];
uniform highp vec3 light_position[MAX_LIGHTS];
uniform highp vec3 light_attenuation[MAX_LIGHTS]; // (constant, linear, quadratic)
uniform highp float light_range[MAX_LIGHTS]; // 0=unlimited
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

// Cube shadow maps of point/spot lights: six layers per light, in the face
// order +X,-X,+Y,-Y,+Z,-Z (see CompiledViewport.cpp)
uniform highp sampler2DArrayShadow pointShadowMapArray;
uniform int light_shadow_index[MAX_LIGHTS]; // cube map of each light, or -1
uniform highp vec2 point_shadow_near_far[MAX_SHADOW_POINT_LIGHTS];

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

// Returns the shadow amount [0,1] of a point/spot light with a cube shadow map.
// geomNormal: the surface normal without normal mapping.
mediump float PointShadowCalculation(
    int lightIdx,
    int cube,
    highp vec3 fragWorldPos,
    mediump vec3 geomNormal)
{
    highp vec3 v = fragWorldPos - light_position[lightIdx];
    highp float texelSize = 1.0 / float(textureSize(pointShadowMapArray, 0).x);

    // Normal offset (about 1.5 shadow map texels at this distance) against
    // shadow acne:
    highp float d = max(abs(v.x), max(abs(v.y), abs(v.z)));
    v += geomNormal * (3.0 * d * texelSize);

    highp vec3 a = abs(v);
    int face;
    highp float ma;
    highp vec3 right;
    highp vec3 up;
    if (a.x >= a.y && a.x >= a.z) {
        ma = a.x;
        up = vec3(0.0, 0.0, 1.0);
        if (v.x > 0.0) { face = 0; right = vec3(0.0, -1.0, 0.0); }
        else           { face = 1; right = vec3(0.0,  1.0, 0.0); }
    } else if (a.y >= a.z) {
        ma = a.y;
        up = vec3(0.0, 0.0, 1.0);
        if (v.y > 0.0) { face = 2; right = vec3( 1.0, 0.0, 0.0); }
        else           { face = 3; right = vec3(-1.0, 0.0, 0.0); }
    } else {
        ma = a.z;
        up = vec3(0.0, 1.0, 0.0);
        if (v.z > 0.0) { face = 4; right = vec3(-1.0, 0.0, 0.0); }
        else           { face = 5; right = vec3( 1.0, 0.0, 0.0); }
    }

    highp float zn = point_shadow_near_far[cube].x;
    highp float zf = point_shadow_near_far[cube].y;
    // Depth of the 90 deg perspective projection of the cube face:
    highp float zNdc = (zf + zn) / (zf - zn) - 2.0 * zf * zn / ((zf - zn) * ma);
    highp float refDepth = zNdc * 0.5 + 0.5;
    if (refDepth >= 1.0)
        return 0.0;

    highp vec2 uv = vec2(dot(right, v), dot(up, v)) / ma * 0.5 + 0.5;
    highp float layer = float(cube * 6 + face);

    // 4 taps of hardware 2x2 bilinear depth comparisons
    mediump float shadow = 0.0;
    for (int x = 0; x < 2; ++x)
    {
        for (int y = 0; y < 2; ++y)
        {
            highp vec2 o = (vec2(float(x), float(y)) - 0.5) * texelSize;
            shadow += 1.0 - texture(pointShadowMapArray, vec4(uv + o, layer, refDepth));
        }
    }
    return shadow * 0.25;
}
)XXX"
