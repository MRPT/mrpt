/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/opengl/RenderableProxy.h>

namespace mrpt::opengl
{
/** GPU-side proxy for rendering lines and wireframes.
 *
 * This proxy handles rendering of any CVisualObject that uses the WIREFRAME shader,
 * such as:
 * - CSetOfLines
 * - CSimpleLine
 * - CGridPlaneXY / CGridPlaneXZ
 * - CAxis
 * - Wireframe rendering of meshes
 * - Any object derived from VisualObjectParams_Lines
 *
 * The proxy manages:
 * - Vertex buffer (line endpoints as pairs)
 * - Color buffer (per-vertex RGBA colors)
 * - Line rendering parameters (width, anti-aliasing)
 * - VAO for efficient attribute binding
 *
 * Line rendering features:
 * - Line width in pixels: widths larger than one pixel are drawn as
 *   screen-aligned quads (see LinesProxyBase)
 * - Per-vertex colors
 * - Optional anti-aliasing of thick lines (edges faded in the fragment shader)
 * - Alpha blending for transparency
 *
 * Typical data flow:
 * 1. compile(): Upload initial line vertices and colors to GPU
 * 2. updateBuffers(): Re-upload when lines change
 * 3. render(): Draw lines every frame
 *
 * \sa LinesProxyBase, mrpt::viz::VisualObjectParams_Lines
 * \ingroup mrpt_opengl_grp
 */
class LinesProxy : public LinesProxyBase
{
 public:
  LinesProxy() = default;

  const char* typeName() const override { return "LinesProxy"; }
};

}  // namespace mrpt::opengl
