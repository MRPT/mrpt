R"XXX(
#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

out highp vec4 color;
in highp vec4 frag_color;

void main()
{
    color = frag_color;
}
)XXX"
