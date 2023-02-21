R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

in vec3 position;
out highp vec3 TexCoords;

uniform highp mat4 pmv_matrix;

void main()
{
    highp vec4 pos = pmv_matrix * vec4(position, 1.0);
    gl_Position = pos.xyww;
    TexCoords = position;
}
)XXX"
