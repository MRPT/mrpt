R"XXX(// #version 300 es (already in the included header)

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects (textured)
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project

// This file will be merged with shadow-calculation.f.glsl
// Light uniforms (num_lights, light_type[], etc.) are declared there.

out lowp vec4 color;

uniform highp vec3 cam_position;
uniform lowp float materialSpecular;  //  [0,1]
uniform highp float materialSpecularExponent;
uniform lowp vec3 materialEmissive;

uniform lowp sampler2D textureSampler;
uniform lowp sampler2D normalMapSampler;

in highp vec3 frag_position, frag_normal;
in mediump vec2 frag_UV; // Interpolated UV texture coords
in highp vec4 frag_posLightSpace;
in highp vec3 frag_tangent;


void main()
{
    highp vec3 N = normalize(frag_normal);

    // Normal mapping via TBN matrix
    highp vec3 T = normalize(frag_tangent);
    T = normalize(T - dot(T, N) * N);
    highp vec3 B = cross(N, T);
    highp mat3 TBN = mat3(T, B, N);

    highp vec3 tangentNormal = texture(normalMapSampler, frag_UV).rgb * 2.0 - 1.0;
    highp vec3 normal = normalize(TBN * tangentNormal);

    highp vec3 cam2frag = cam_position - frag_position;
    mediump float cam2fragDist = length(cam2frag);
    highp vec3 viewDirection = normalize(cam2frag);

    // Hemisphere ambient
    mediump vec3 ambientColor = mix(ambient_ground_color, ambient_sky_color, 0.5 + 0.5 * normal.z);
    mediump vec3 totalDiffuse = light_ambient * ambientColor;
    mediump vec3 totalSpecular = vec3(0.0);

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

        highp vec3 halfVector = normalize(viewDirection + lightDir);
        highp float specAmount = pow(max(dot(normal, halfVector), 0.0), materialSpecularExponent);
        mediump float specular_factor = (diff > 0.0) ? specAmount * materialSpecular * light_specular[i] : 0.0;

        // Shadow only applies to the primary directional light (index 0)
        mediump float shadowFactor = 1.0;
        if (i == 0 && light_type[i] == 0) {
            mediump float shadow = ShadowCalculation(frag_posLightSpace, normal, cam2fragDist);
            shadowFactor = 1.0 - shadow;
        }

        totalDiffuse += attenuation * shadowFactor * diff * light_diffuse[i] * light_color[i];
        totalSpecular += attenuation * shadowFactor * specular_factor * light_color[i];
    }

    // material texture color:
    lowp vec4 texCol = texture(textureSampler,frag_UV);

    mediump vec3 litColor = materialEmissive + texCol.rgb * totalDiffuse + totalSpecular;

    if (fog_enabled) {
        highp float dist = cam2fragDist;
        mediump float fogFactor;
        if (fog_mode == 1)
            fogFactor = exp(-fog_density * dist);
        else if (fog_mode == 2)
            fogFactor = exp(-fog_density * fog_density * dist * dist);
        else
            fogFactor = clamp((fog_far - dist) / (fog_far - fog_near), 0.0, 1.0);
        litColor = mix(fog_color, litColor, fogFactor);
    }

    color = vec4(litColor, texCol.a);
}
)XXX"
