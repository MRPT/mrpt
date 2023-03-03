R"XXX(#version 300 es

// FRAGMENT SHADER: Default shader for MRPT CRenderizable objects
// Jose Luis Blanco Claraco (C) 2019-2023
// Part of the MRPT project

out mediump vec4 FragColor;
in mediump vec2 frag_UV;
uniform sampler2D textureId;

highp float near_plane = 0.1;
highp float far_plane = 30.0;

// required when using a perspective projection matrix
highp float LinearizeDepth(highp float depth)
{
    highp float z = depth * 2.0 - 1.0; // Back to NDC 
    return (2.0 * near_plane * far_plane) / (far_plane + near_plane - z * (far_plane - near_plane));	
}

void main()
{
    highp float depthValue = texture(textureId, frag_UV).r;
    // FragColor = vec4(vec3(LinearizeDepth(depthValue) / far_plane), 1.0); // perspective
    FragColor = vec4(vec3(depthValue), 1.0); // orthographic
}
)XXX"
