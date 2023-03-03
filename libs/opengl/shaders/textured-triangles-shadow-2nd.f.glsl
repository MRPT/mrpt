R"XXX(// #version 300 es (already in the included header)

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

// This file will be merged with shadow-calculation.f.glsl

out lowp vec4 color;

uniform lowp vec3 light_color;
uniform mediump float light_ambient, light_diffuse, light_specular;
//uniform highp vec3 light_direction; // already in #include

uniform highp vec3 cam_position;
uniform lowp float materialSpecular;  //  [0,1]

uniform lowp sampler2D textureSampler;

in highp vec3 frag_position, frag_normal;
in mediump vec2 frag_UV; // Interpolated UV texture coords
in highp vec4 frag_posLightSpace;


void main()
{
    // diffuse lighting
    mediump vec3 normal = normalize(frag_normal);
    mediump float diff = max(dot(normal, -light_direction), 0.0f);
    mediump float diffuse_factor = diff * light_diffuse;

    // specular lighting
    highp vec3 cam2frag = cam_position - frag_position;
    mediump float cam2fragDist = length(cam2frag);
    highp vec3 viewDirection = normalize(cam2frag);
    highp vec3 reflectionDirection = reflect(light_direction, normal);
    mediump float specAmount = pow(max(dot(viewDirection, reflectionDirection), 0.0f), 16.0f);
    mediump float specular = specAmount * materialSpecular;
    mediump float specular_factor = specAmount * materialSpecular * light_specular;
 
    // calculate shadow
    mediump float shadow = ShadowCalculation(frag_posLightSpace, normal, cam2fragDist);
    mediump vec3 finalLight = (light_ambient + (1.0 - shadow)*(diffuse_factor+specular_factor))*light_color;

    // material texture color:
    lowp vec4 texCol = texture(textureSampler,frag_UV);
    
    color = vec4(texCol.rgb * finalLight, texCol.a);
}
)XXX"
