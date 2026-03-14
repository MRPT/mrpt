R"XXX(// #version 300 es (already in the included header)

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project

// This file will be merged with shadow-calculation.f.glsl
// Light uniforms (num_lights, light_type[], etc.) are declared there.

out lowp vec4 color;

uniform highp vec3 cam_position;
uniform lowp float materialSpecular;  //  [0,1]
uniform highp float materialSpecularExponent;
uniform lowp vec3 materialEmissive;

// JLBC: Was "struct" Frag .... but that requires #version 320 es. Let's keep it minimum.
in highp vec3 frag_position, frag_normal;
in lowp vec4 frag_materialColor;
in highp vec4 frag_posLightSpace;

void main()
{
    highp vec3 normal = normalize(frag_normal);
    highp vec3 cam2frag = cam_position - frag_position;
    mediump float cam2fragDist = length(cam2frag);
    highp vec3 viewDirection = normalize(cam2frag);

    mediump vec3 totalLight = vec3(light_ambient);

    for (int i = 0; i < num_lights; i++)
    {
        highp vec3 lightDir;
        mediump float attenuation = 1.0;

        if (light_type[i] == 0) {
            lightDir = -light_direction[i];
        } else {
            highp vec3 toLight = light_position[i] - frag_position;
            highp float dist = length(toLight);
            lightDir = toLight / dist;
            attenuation = 1.0 / (light_attenuation[i].x + light_attenuation[i].y * dist + light_attenuation[i].z * dist * dist);

            if (light_type[i] == 2) {
                mediump float theta = dot(lightDir, -light_direction[i]);
                mediump float epsilon = light_spot_cutoff[i].x - light_spot_cutoff[i].y;
                mediump float spotIntensity = clamp((theta - light_spot_cutoff[i].y) / epsilon, 0.0, 1.0);
                attenuation *= spotIntensity;
            }
        }

        highp float diff = max(dot(normal, lightDir), 0.0);
        highp float diffuse_factor = diff * light_diffuse[i];

        highp vec3 halfVector = normalize(viewDirection + lightDir);
        highp float specAmount = pow(max(dot(normal, halfVector), 0.0), materialSpecularExponent);
        mediump float specular_factor = (diff > 0.0) ? specAmount * materialSpecular * light_specular[i] : 0.0;

        // Shadow only applies to the primary directional light (index 0)
        mediump float shadowFactor = 1.0;
        if (i == 0 && light_type[i] == 0) {
            mediump float shadow = ShadowCalculation(frag_posLightSpace, normal, cam2fragDist);
            shadowFactor = 1.0 - shadow;
        }

        totalLight += attenuation * shadowFactor * (diffuse_factor + specular_factor) * light_color[i];
    }

    color = vec4(materialEmissive + frag_materialColor.rgb * totalLight, frag_materialColor.a);
}
)XXX"
