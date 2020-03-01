R"XXX(
#version 330 core

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2020
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;

uniform mat4 p_matrix;
uniform mat4 mv_matrix;
uniform float vertexPointSize;
uniform int enableVariablePointSize;  // 0 or 1
uniform float variablePointSize_K, variablePointSize_DepthScale;

out vec4 frag_color;

void main()
{
    vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;
    if (enableVariablePointSize!=0)
      gl_PointSize = vertexPointSize +
		variablePointSize_K/(variablePointSize_DepthScale*gl_Position.z + 0.01);
    else
	  gl_PointSize = vertexPointSize;

    frag_color = vertexColor;
}
)XXX"
