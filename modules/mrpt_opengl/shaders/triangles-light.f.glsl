R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

uniform lowp vec3 light_color;
uniform mediump float light_ambient, light_diffuse, light_specular;
uniform highp vec3 light_direction;

uniform highp vec3 cam_position;
uniform lowp float materialSpecular;

in highp vec3 frag_position, frag_normal;
in lowp vec4 frag_materialColor;

out lowp vec4 color;

void main()
{
    // diffuse lighting
    highp vec3 normal = normalize(frag_normal);
    highp float diff = max(dot(normal, -light_direction), 0.0f);
    highp float diffuse_factor = diff * light_diffuse;

    // specular lighting
    highp vec3 viewDirection = normalize(cam_position - frag_position);
    highp vec3 reflectionDirection = reflect(light_direction, normal);
    highp float specAmount = pow(max(dot(viewDirection, reflectionDirection), 0.0f), 16.0f);
    mediump float specular_factor = specAmount * materialSpecular * light_specular;

    mediump vec3 finalLight = (light_ambient + (diffuse_factor+specular_factor))*light_color;

    color = vec4(frag_materialColor.rgb * finalLight, frag_materialColor.a);
}
)XXX"
