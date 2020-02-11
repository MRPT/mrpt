R"XXX(
#version 330 core

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project

out vec4 color;
in vec4 frag_color;

void main()
{
    color = frag_color;
}
)XXX"
