R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

uniform lowp vec4 light_diffuse, light_ambient, light_specular;
uniform highp vec3 light_direction;
uniform highp vec3 cam_position;
uniform lowp float materialSpecular;

in Fragment {
    highp vec3 position, normal;
    lowp vec4 materialColor;
} frag;

out lowp vec4 color;

void main()
{
    // diffuse lighting
    highp vec3 normal = normalize(frag.normal);
    highp float diff = max(dot(normal, -light_direction), 0.0f);
    highp vec4 diffuse_factor = diff * light_diffuse;

    // specular lighting
    highp vec3 viewDirection = normalize(cam_position - frag.position);
    highp vec3 reflectionDirection = reflect(light_direction, normal);
    highp float specAmount = pow(max(dot(viewDirection, reflectionDirection), 0.0f), 16.0f);
    highp float specular = specAmount * materialSpecular;

    color = frag.materialColor * (diffuse_factor + light_ambient) + specular * light_specular;
}
)XXX"
