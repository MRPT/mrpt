R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project


in vec3 position;
in vec2 vertexUV;

uniform highp mat4 pmv_matrix;

out mediump vec2 frag_UV; // Interpolated UV texture coords

void main()
{
    frag_UV = vertexUV;

    gl_Position = pmv_matrix * vec4(position, 1.0);
}
)XXX"
