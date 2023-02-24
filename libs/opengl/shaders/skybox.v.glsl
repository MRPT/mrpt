R"XXX(#version 300 es

// VERTEX SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

in vec3 position;
out highp vec3 TexCoords;

uniform highp mat4 p_matrix, v_matrix_no_translation;

void main()
{
    highp vec4 pos = p_matrix * v_matrix_no_translation * vec4(position, 1.0);
    // (x,y,w,w), such that depth z/w=1
    gl_Position = pos.xyww;
    
    // This change of coordinates is required to have sky boxes in their place:
    TexCoords = vec3(position.x,position.z,position.y);
}
)XXX"
