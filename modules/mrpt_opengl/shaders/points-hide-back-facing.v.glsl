R"XXX(#version 300 es

// VERTEX SHADER: Like points.v.glsl, but hiding points whose direction vector
// faces away from the camera.
// Part of the MRPT project

layout(location = 0) in vec3 position;
layout(location = 1) in vec4 vertexColor;
layout(location = 2) in vec3 direction;

uniform highp mat4 p_matrix;
uniform highp mat4 mv_matrix;
uniform float vertexPointSize;
uniform int enableVariablePointSize;  // 0 or 1
uniform float variablePointSize_K, variablePointSize_DepthScale;
uniform float backFacingMinCos;  // cos(max visible angle)

out lowp vec4 frag_color;

void main()
{
    highp vec4 eye_position = mv_matrix * vec4(position, 1.0);

    // Direction towards the camera, in eye coordinates. Orthographic
    // projections have p_matrix[3][3]=1 and a constant viewing direction:
    highp vec3 to_camera = (p_matrix[3][3] == 1.0) ?
        vec3(0.0, 0.0, 1.0) : normalize(-eye_position.xyz);
    highp vec3 dir_eye = mat3(mv_matrix) * direction;
    highp float len = length(dir_eye);

    if (len > 0.0 && dot(dir_eye, to_camera) < backFacingMinCos * len)
    {
        // Outside of the clip volume: discarded before rasterization.
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        gl_PointSize = 1.0;
        frag_color = vertexColor;
        return;
    }

    gl_Position = p_matrix * eye_position;
    if (enableVariablePointSize!=0)
      gl_PointSize = vertexPointSize +
		variablePointSize_K/(variablePointSize_DepthScale*abs(gl_Position.z) + 0.01);
    else
	  gl_PointSize = vertexPointSize;

    frag_color = vertexColor;
}
)XXX"
