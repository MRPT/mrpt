R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;
layout(location = 3) in vec2 vertexUV;

uniform highp mat4 pmv_matrix;

out mediump vec2 frag_UV; // Interpolated UV texture coords
out lowp vec4 frag_vertexColor;

void main()
{
    frag_UV = vertexUV;
    frag_vertexColor = vertexColor;

    gl_Position = pmv_matrix * vec4(position, 1.0);
}
)XXX"
