R"XXX(#version 300 es

// FRAGMENT SHADER: SSAO geometry pre-pass — writes view-space pos and normal to MRT
// Jose Luis Blanco Claraco (C) 2026
// Part of the MRPT project

in highp vec3 frag_viewPos;
in highp vec3 frag_viewNormal;

layout(location = 0) out highp vec4 gPosition;
layout(location = 1) out highp vec4 gNormal;

void main()
{
    gPosition = vec4(frag_viewPos, 1.0);  // w=1 marks valid pixel
    gNormal   = vec4(normalize(frag_viewNormal), 0.0);
}
)XXX"
