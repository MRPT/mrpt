R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 vertexNormal;
layout(location = 3) in vec2 vertexUV;
layout(location = 4) in vec3 vertexTangent;

uniform highp mat4 p_matrix;
uniform highp mat4 v_matrix;
uniform highp mat4 m_matrix;

out highp vec3 frag_position, frag_normal;
out mediump vec2 frag_UV; // Interpolated UV texture coords
out lowp vec4 frag_vertexColor;
out highp vec3 frag_tangent;

void main()
{
    frag_UV = vertexUV;
    frag_vertexColor = vertexColor;

    highp mat3 normalMatrix = mat3(m_matrix);
    frag_position = vec3(m_matrix * vec4(position, 1.0));
    frag_normal   = normalize(normalMatrix * vertexNormal);

    // Transform tangent to world space (if tangent is valid)
    frag_tangent = normalize(normalMatrix * vertexTangent);

    gl_Position = p_matrix * v_matrix * vec4(frag_position, 1.0);
}

)XXX"
