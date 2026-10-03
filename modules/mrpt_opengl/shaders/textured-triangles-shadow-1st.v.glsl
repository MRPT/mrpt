R"XXX(#version 300 es

// VERTEX SHADER: shadow map pass for textured triangles with alpha cutout
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project

layout(location = 0) in vec3 position;
layout(location = 3) in vec2 vertexUV;

uniform highp mat4 m_matrix;
uniform highp mat4 light_pv_matrix; // =p*v matrices

out mediump vec2 frag_UV;

void main()
{
    frag_UV = vertexUV;
    gl_Position = light_pv_matrix * m_matrix * vec4(position, 1.0);
}
)XXX"
