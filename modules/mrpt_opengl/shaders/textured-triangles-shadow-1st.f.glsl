R"XXX(#version 300 es

// FRAGMENT SHADER: shadow map pass for textured triangles with alpha cutout
// Jose Luis Blanco Claraco (C) 2019-2026
// Part of the MRPT project

uniform lowp sampler2D textureSampler;
uniform mediump float alphaCutoff;

in mediump vec2 frag_UV;

void main()
{
    // Cut out fragments do not cast shadows:
    if (texture(textureSampler, frag_UV).a < alphaCutoff)
    {
        discard;
    }
}
)XXX"
