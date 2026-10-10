R"XXX(#version 300 es

// VERTEX SHADER: Full-screen triangle for the CFBORender RGB post-processing
// pass (lens distortion and pixel noise)
// Jose Luis Blanco Claraco (C) 2026
// Part of the MRPT project

void main()
{
    mediump vec2 pos = vec2(
        float((gl_VertexID & 1) << 2) - 1.0,
        float((gl_VertexID & 2) << 1) - 1.0
    );
    gl_Position = vec4(pos, 0.0, 1.0);
}
)XXX"
