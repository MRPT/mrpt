R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

uniform lowp vec3 light_color;
uniform mediump float light_ambient, light_diffuse, light_specular;
uniform highp vec3 light_direction;


uniform lowp sampler2D textureSampler;
uniform highp vec3 cam_position;
uniform lowp float materialSpecular;

in highp vec3 frag_position, frag_normal;
in mediump vec2 frag_UV; // Interpolated values from the vertex shaders

out lowp vec4 color;

void main()
{
    // diffuse lighting
    mediump vec3 normal = normalize(frag_normal);
    mediump float diff = max(dot(normal, -light_direction), 0.0);
    mediump float diffuse_factor = diff * light_diffuse;
    
    // specular lighting
    highp vec3 viewDirection = normalize(cam_position - frag_position);
    highp vec3 reflectionDirection = reflect(light_direction, normal);
    mediump float specAmount = pow(max(dot(viewDirection, reflectionDirection), 0.0f), 16.0f);
    mediump float specular_factor = specAmount * materialSpecular * light_specular;

    // material texture color:
    lowp vec4 texCol = texture(textureSampler,frag_UV);
    mediump vec3 finalLight = (light_ambient + (diffuse_factor+specular_factor))*light_color;
    
    color = vec4(texCol.rgb * finalLight, texCol.a);
}

)XXX"
