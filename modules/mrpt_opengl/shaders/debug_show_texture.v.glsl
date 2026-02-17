R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

layout(location = 0) in vec3 position;
layout(location = 3) in vec2 vertexUV;

out mediump vec2 frag_UV; // Interpolated UV texture coords

void main()
{
    frag_UV = vertexUV;
    gl_Position = vec4(position, 1.0);
}
)XXX"
