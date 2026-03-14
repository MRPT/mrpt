R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project

// Multi-light support (up to 8 lights)
#define MAX_LIGHTS 8

uniform int num_lights;
uniform int light_type[MAX_LIGHTS];       // 0=directional, 1=point, 2=spot
uniform lowp vec3 light_color[MAX_LIGHTS];
uniform mediump float light_diffuse[MAX_LIGHTS];
uniform mediump float light_specular[MAX_LIGHTS];
uniform highp vec3 light_direction[MAX_LIGHTS];
uniform highp vec3 light_position[MAX_LIGHTS];
uniform highp vec3 light_attenuation[MAX_LIGHTS]; // (constant, linear, quadratic)
uniform mediump vec2 light_spot_cutoff[MAX_LIGHTS]; // (cos_inner, cos_outer)

uniform mediump float light_ambient;
uniform lowp vec3 ambient_sky_color;
uniform lowp vec3 ambient_ground_color;

uniform highp vec3 cam_position;
uniform lowp float materialSpecular;
uniform highp float materialSpecularExponent;
uniform lowp vec3 materialEmissive;

uniform bool fog_enabled;
uniform lowp vec3 fog_color;
uniform highp float fog_near;
uniform highp float fog_far;
uniform int fog_mode;       // 0=linear, 1=exp, 2=exp²
uniform highp float fog_density;

in highp vec3 frag_position, frag_normal;
in lowp vec4 frag_materialColor;

out lowp vec4 color;

void main()
{
    highp vec3 normal = normalize(frag_normal);
    highp vec3 viewDirection = normalize(cam_position - frag_position);

    // Hemisphere ambient: blend sky/ground color based on world-space normal.z
    mediump vec3 ambientColor = mix(ambient_ground_color, ambient_sky_color, 0.5 + 0.5 * normal.z);
    mediump vec3 totalDiffuse = light_ambient * ambientColor;
    mediump vec3 totalSpecular = vec3(0.0);

    for (int i = 0; i < num_lights; i++)
    {
        highp vec3 lightDir;
        mediump float attenuation = 1.0;

        if (light_type[i] == 0) {
            // Directional light
            lightDir = -light_direction[i];
        } else {
            // Point or Spot light
            highp vec3 toLight = light_position[i] - frag_position;
            highp float dist = length(toLight);
            lightDir = toLight / dist;
            attenuation = 1.0 / (light_attenuation[i].x + light_attenuation[i].y * dist + light_attenuation[i].z * dist * dist);

            if (light_type[i] == 2) {
                // Spot light
                mediump float theta = dot(lightDir, -light_direction[i]);
                mediump float epsilon = light_spot_cutoff[i].x - light_spot_cutoff[i].y;
                mediump float spotIntensity = clamp((theta - light_spot_cutoff[i].y) / epsilon, 0.0, 1.0);
                attenuation *= spotIntensity;
            }
        }

        // Diffuse
        highp float diff = max(dot(normal, lightDir), 0.0);
        totalDiffuse += attenuation * diff * light_diffuse[i] * light_color[i];

        // Specular (Blinn-Phong) — added as white, not tinted by material
        highp vec3 halfVector = normalize(viewDirection + lightDir);
        highp float specAmount = pow(max(dot(normal, halfVector), 0.0), materialSpecularExponent);
        mediump float specular_factor = (diff > 0.0) ? specAmount * materialSpecular * light_specular[i] : 0.0;
        totalSpecular += attenuation * specular_factor * light_color[i];
    }

    mediump vec3 litColor = materialEmissive + frag_materialColor.rgb * totalDiffuse + totalSpecular;

    if (fog_enabled) {
        highp float dist = length(cam_position - frag_position);
        mediump float fogFactor;
        if (fog_mode == 1)
            fogFactor = exp(-fog_density * dist);
        else if (fog_mode == 2)
            fogFactor = exp(-fog_density * fog_density * dist * dist);
        else
            fogFactor = clamp((fog_far - dist) / (fog_far - fog_near), 0.0, 1.0);
        litColor = mix(fog_color, litColor, fogFactor);
    }

    color = vec4(litColor, frag_materialColor.a);
}
)XXX"
