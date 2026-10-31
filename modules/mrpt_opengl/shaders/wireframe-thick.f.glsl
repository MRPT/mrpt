R"XXX(#version 300 es

// FRAGMENT SHADER: lines wider than one pixel (see wireframe-thick.v.glsl)
// Part of the MRPT project

in lowp vec4 frag_color;
in highp float frag_dist;

uniform highp float line_width;      // visible width, in pixels
uniform highp float line_aa_margin;  // 0: hard edges, >0: anti-aliased edges

out lowp vec4 color;

void main()
{
    color = frag_color;
    if (line_aa_margin > 0.0)
    {
        // Coverage of a 1-pixel wide footprint across the line edge:
        color.a *= clamp(0.5 * line_width + 0.5 - abs(frag_dist), 0.0, 1.0);
    }
}
)XXX"
