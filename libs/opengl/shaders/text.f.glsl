R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

in lowp vec4 frag_color;
out lowp vec4 color;

void main()
{
    color = frag_color;
}
)XXX"
