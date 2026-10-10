R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project


layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;
layout(location = 5) in highp mat4 instanceMatrix;  // identity unless instanced (INSTANCE_MATRIX_ATTRIB_LOCATION)

uniform highp mat4 pmv_matrix;

out lowp vec4 frag_materialColor;

void main()
{
    gl_Position = pmv_matrix * (instanceMatrix * vec4(position, 1.0));
    frag_materialColor = vertexColor;
}
)XXX"
