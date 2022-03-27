R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


in vec3 position;
in vec2 vertexUV;
in vec3 vertexNormal;

uniform mediump mat4 p_matrix;
uniform mediump mat4 mv_matrix;
uniform mediump mat4 pmv_matrix;

out mediump vec3 frag_position, frag_normal;
out mediump vec2 frag_UV; // Interpolated UV texture coords
uniform lowp int enableLight;  // 0 or 1

void main()
{
    frag_UV = vertexUV;

    if (enableLight!=0)
    {
        mediump vec4 eye_position = mv_matrix * vec4(position, 1.0);
        gl_Position = p_matrix * eye_position;
        frag_position = eye_position.xyz;
        frag_normal   = (mv_matrix * vec4(normalize(vertexNormal), 0.0)).xyz;
    }
    else
    {
        gl_Position = pmv_matrix * vec4(position, 1.0);
    }
}
)XXX"
