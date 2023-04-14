R"XXX(#version 300 es

// "#include" file for shadow calculation in fragment shaders (2nd pass)
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

uniform highp sampler2D shadowMap;
uniform highp vec3 light_direction;

uniform highp float shadow_bias, shadow_bias_cam2frag, shadow_bias_normal;

mediump float ShadowCalculation(
    highp vec4 fragPosLightSpace,
    mediump vec3 normal,
    mediump float cam2fragDist)
{
   // perform perspective divide
    highp vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;

    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;

    if(projCoords.z > 1.0) return 0.0;

    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    highp float closestDepth = texture(shadowMap, projCoords.xy).r;
    // get depth of current fragment from light's perspective
    highp float currentDepth = projCoords.z;
    
    // check whether current frag pos is in shadow1D
    highp float bias = shadow_bias + shadow_bias_cam2frag*cam2fragDist + shadow_bias_normal*(1.0-max(0.0,dot(normal, light_direction)));
#if 0
    mediump float shadow = currentDepth-bias > closestDepth  ? 1.0 : 0.0;
#else
    mediump float shadow = 0.0f;
    mediump vec2 texelSize = 1.0 / vec2(textureSize(shadowMap, 0));
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            highp float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r; 
            shadow += currentDepth - bias > pcfDepth  ? 1.0 : 0.0;
        }
    }
    shadow /= 9.0;
#endif
    return shadow;
}
)XXX"
