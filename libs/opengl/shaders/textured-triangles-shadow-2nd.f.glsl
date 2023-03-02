R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

out lowp vec4 color;

uniform lowp vec3 light_color;
uniform mediump float light_ambient, light_diffuse, light_specular;
uniform highp vec3 light_direction;

uniform highp vec3 cam_position;
uniform lowp float materialSpecular;  //  [0,1]

uniform lowp sampler2D textureSampler;
uniform highp sampler2D shadowMap;

in highp vec3 frag_position, frag_normal;
in mediump vec2 frag_UV; // Interpolated UV texture coords
in highp vec4 frag_posLightSpace;


mediump float ShadowCalculation(vec4 fragPosLightSpace, vec3 normal)
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

    // check whether current frag pos is in shadow
    highp float bias = max(0.05 * (1.0 - dot(normal, -light_direction)), 0.005);
#if 0
    highp float shadow = currentDepth-bias > closestDepth  ? 1.0 : 0.0;
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

void main()
{
    // diffuse lighting
    mediump vec3 normal = normalize(frag_normal);
    mediump float diff = max(dot(normal, -light_direction), 0.0f);
    mediump float diffuse_factor = diff * light_diffuse;

    // specular lighting
    highp vec3 viewDirection = normalize(cam_position - frag_position);
    highp vec3 reflectionDirection = reflect(light_direction, normal);
    mediump float specAmount = pow(max(dot(viewDirection, reflectionDirection), 0.0f), 16.0f);
    mediump float specular = specAmount * materialSpecular;
    mediump float specular_factor = specAmount * materialSpecular * light_specular;
 
    // calculate shadow
    mediump float shadow = ShadowCalculation(frag_posLightSpace, normal);
    mediump vec3 finalLight = (light_ambient + (1.0 - shadow)*(diffuse_factor+specular_factor))*light_color;

    // material texture color:
    lowp vec4 texCol = texture(textureSampler,frag_UV);
    
    color = vec4(texCol.rgb * finalLight, texCol.a);
}
)XXX"
