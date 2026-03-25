R"XXX(#version 300 es

// VERTEX SHADER: Full-screen triangle for SSAO blur pass
// Jose Luis Blanco Claraco (C) 2026
// Part of the MRPT project

out mediump vec2 frag_UV;

void main()
{
    mediump vec2 pos = vec2(
        float((gl_VertexID & 1) << 2) - 1.0,
        float((gl_VertexID & 2) << 1) - 1.0
    );
    frag_UV     = pos * 0.5 + 0.5;
    gl_Position = vec4(pos, 0.0, 1.0);
}
)XXX"
