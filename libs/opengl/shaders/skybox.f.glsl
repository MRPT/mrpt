R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

in highp vec3 TexCoords;
out lowp vec4 color;

uniform samplerCube skybox;

void main()
{
	color = texture(skybox, TexCoords);
	//color = vec4(0.2,0.2,0.8, 1.0);
}
)XXX"
