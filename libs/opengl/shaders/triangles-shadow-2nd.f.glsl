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

uniform highp sampler2D shadowMap;

in Fragment {
    highp vec3 position, normal;
    lowp vec4 materialColor;
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
    highp float closestDepth = texture(shadowMap, projCoords.xy).r;
    // get depth of current fragment from light's perspective
    highp float currentDepth = projCoords.z;
    // check whether current frag pos is in shadow
    highp float bias = 0.005;
    mediump float shadow = currentDepth > closestDepth  ? 1.0 : 0.0;

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

    highp vec3 projCoords = 0.5*(frag.posLightSpace.xyz+vec3(1,1,1)) / frag.posLightSpace.w;
    // transform to [0,1] range
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    highp float closestDepth = texture(shadowMap, projCoords.xy).r;
    
    color = 0.0001*frag.materialColor * (diffuse_factor + (1.0 - shadow)*(light_ambient + specular_factor));
    + vec4(texture(shadowMap, projCoords.xy).xyz, 1);
}
)XXX"
