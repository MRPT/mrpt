R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

out lowp vec4 color;

// TODO: Refactor as struct and single vec3 light color!!
uniform lowp vec4 light_diffuse, light_ambient, light_specular;
uniform highp vec3 light_direction;
uniform highp vec3 cam_position;
uniform lowp float materialSpecular;  //  [0,1]

uniform lowp sampler2D textureSampler;
uniform lowp sampler2D shadowMap;

in Fragment {
    highp vec3 position, normal;
    mediump vec2 UV; // Interpolated UV texture coords
    highp vec4 posLightSpace;
} frag;


mediump float ShadowCalculation(vec4 fragPosLightSpace)
{
   // perform perspective divide
    highp vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;

    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;

    if(projCoords.z > 1.0) return 0.0;

    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    mediump float closestDepth = texture(shadowMap, projCoords.xy).r;
    // get depth of current fragment from light's perspective
    mediump float currentDepth = projCoords.z;

    // check whether current frag pos is in shadow
    mediump float bias = 0.005;
#if 0
    mediump float shadow = currentDepth-bias > closestDepth  ? 1.0 : 0.0;
#else
    mediump float shadow = 0.0f;
    mediump vec2 texelSize = 1.0 / vec2(textureSize(shadowMap, 0));
    for(int x = -1; x <= 1; ++x)
    {
        for(int y = -1; y <= 1; ++y)
        {
            mediump float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r; 
            shadow += currentDepth - bias > pcfDepth  ? 1.0 : 0.0;
        }
    }
    shadow /= 9.0;
#endif
    return shadow;
}

void main()
{
    // diffuse lighting
    mediump vec3 normal = normalize(frag.normal);
    mediump float diff = max(dot(normal, -light_direction), 0.0f);
    mediump vec4 diffuse_factor = diff * light_diffuse;

    // specular lighting
    highp vec3 viewDirection = normalize(cam_position - frag.position);
    highp vec3 reflectionDirection = reflect(light_direction, normal);
    mediump float specAmount = pow(max(dot(viewDirection, reflectionDirection), 0.0f), 16.0f);
    mediump float specular = specAmount * materialSpecular;
    mediump vec4 specular_factor = specular * light_specular;
 
    // calculate shadow
    mediump float shadow = ShadowCalculation(frag.posLightSpace);
    
    color = texture( textureSampler, frag.UV ) * vec4((vec3(light_ambient) + (1.0 - shadow)*(vec3(diffuse_factor + specular_factor))), 1);
}
)XXX"
