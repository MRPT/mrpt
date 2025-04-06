R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project


in vec3 position;
in vec4 vertexColor;

uniform highp mat4 p_matrix;
uniform highp mat4 mv_matrix;
uniform float vertexPointSize;
uniform int enableVariablePointSize;  // 0 or 1
uniform float variablePointSize_K, variablePointSize_DepthScale;

out lowp vec4 frag_color;

void main()
{
    highp vec4 eye_position = mv_matrix * vec4(position, 1.0);
    gl_Position = p_matrix * eye_position;
    if (enableVariablePointSize!=0)
      gl_PointSize = vertexPointSize +
		variablePointSize_K/(variablePointSize_DepthScale*abs(gl_Position.z) + 0.01);
    else
	  gl_PointSize = vertexPointSize;

    frag_color = vertexColor;
}
)XXX"
