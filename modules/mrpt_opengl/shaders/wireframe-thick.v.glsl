R"XXX(#version 300 es

// VERTEX SHADER: lines wider than one pixel, drawn as one screen-aligned quad
// per segment (4-vertex triangle strip, instanced once per segment). Both
// endpoints are per-instance attributes read from the same buffers as the
// GL_LINES path, so no extra vertex data is stored.
// Part of the MRPT project

layout(location = 0) in highp vec3 position0;
layout(location = 1) in lowp vec4 vertexColor0;
layout(location = 2) in highp vec3 position1;
layout(location = 3) in lowp vec4 vertexColor1;

uniform highp mat4 p_matrix;
uniform highp mat4 mv_matrix;
uniform highp vec2 viewport_size;  // in pixels
uniform highp float line_width;    // visible width, in pixels
uniform highp float line_aa_margin;  // extra half-width for anti-aliasing (0 or 1 px)

out lowp vec4 frag_color;
out highp float frag_dist;  // signed distance to the segment axis, pixels

void main()
{
    // Quad corners: vertex 0,1 at position0; 2,3 at position1; even/odd on
    // each side of the axis.
    highp float t = float(gl_VertexID >> 1);
    highp float side = float(gl_VertexID & 1) * 2.0 - 1.0;

    highp vec4 a = p_matrix * (mv_matrix * vec4(position0, 1.0));
    highp vec4 b = p_matrix * (mv_matrix * vec4(position1, 1.0));

    // Clip the segment against the near plane (z = -w), so the perspective
    // division below never crosses w = 0:
    highp float da = a.z + a.w;
    highp float db = b.z + b.w;
    if (da < 0.0 && db < 0.0)
    {
        gl_Position = vec4(0.0, 0.0, 2.0, 1.0);  // all behind: degenerate, culled
        frag_color = vertexColor0;
        frag_dist = 0.0;
        return;
    }
    if (da < 0.0)
    {
        a = mix(a, b, da / (da - db));
    }
    else if (db < 0.0)
    {
        b = mix(b, a, db / (db - da));
    }

    highp vec2 halfVp = 0.5 * viewport_size;
    highp vec2 d = (b.xy / b.w - a.xy / a.w) * halfVp;
    highp float len = length(d);
    highp vec2 dir = len > 1e-6 ? d / len : vec2(1.0, 0.0);
    highp vec2 n = vec2(-dir.y, dir.x);

    highp float hw = 0.5 * line_width + line_aa_margin;

    // Square caps (extend by half the width) close the gaps at polyline
    // joints. They extend the segment along its own 3D direction (a linear
    // extrapolation in clip space), so the quad keeps the depth of the line
    // at each pixel: moving the corners on screen only would shift its depth
    // and let coplanar geometry show through it.
    highp vec4 c = t < 0.5 ? a : b;
    if (len > 1e-3)
    {
        highp float k = hw / len;
        highp vec4 cExt = t < 0.5 ? mix(a, b, -k) : mix(a, b, 1.0 + k);
        if (cExt.z + cExt.w > 0.0)  // not behind the near plane
        {
            c = cExt;
        }
    }
    else
    {
        // A segment seen end-on (or of zero length): a square dot.
        c.xy += (2.0 * t - 1.0) * hw * dir / halfVp * c.w;
    }
    c.xy += side * hw * n / halfVp * c.w;

    // The quad faces the camera, with the depth of the segment axis all across
    // its width: where it lies on a surface (e.g. a path on the ground) part
    // of it would be hidden below that surface. Draw it nearer to the camera
    // by 2*hw pixels (in world units at the depth of the segment), which keeps
    // lines on surfaces seen at more than ~27 degrees from grazing whole.
    // That is a constant shift of the clip z: coef*2*lift/(p11*height), with
    // coef = p_matrix[3][2] for perspective and p_matrix[2][2] for orthographic
    // projections (p_matrix[2][3] is -1 and 0, respectively). Both are
    // negative; anything else (e.g. the identity of CCamera::setNoProjection())
    // has no camera to get nearer to, and gets no shift.
    highp float lift = 2.0 * hw;
    highp float depthCoef =
        max(0.0, -mix(p_matrix[2][2], p_matrix[3][2], abs(p_matrix[2][3])));
    highp float p11 = max(abs(p_matrix[1][1]), 1e-6);
    c.z -= depthCoef * 2.0 * lift / (p11 * viewport_size.y);

    gl_Position = c;

    frag_color = t < 0.5 ? vertexColor0 : vertexColor1;
    frag_dist = side * hw;
}
)XXX"
