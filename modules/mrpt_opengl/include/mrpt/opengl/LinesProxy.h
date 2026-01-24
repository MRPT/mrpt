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
 * - Variable line width
 * - Per-vertex colors
 * - Optional anti-aliasing (GL_LINE_SMOOTH)
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

  /** @name RenderableProxy Interface Implementation
   * @{ */

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  const char* typeName() const override { return "LinesProxy"; }

  /** @} */

 private:
  /** Cached line rendering parameters */
  struct LineParams
  {
    float lineWidth = 1.0f;
    bool antiAliasing = false;
  };

  /** Cached parameters from last compile/update */
  mutable LineParams m_params;

  /** Helper: Extract line rendering parameters from source object */
  void extractLineParams(const mrpt::viz::CVisualObject* sourceObj);

  /** Helper: Upload line-specific uniforms to shader */
  void uploadLineUniforms(const RenderContext& rc) const;

  /** Helper: Setup OpenGL line state (width, anti-aliasing) */
  void setupLineState() const;

  /** Helper: Restore OpenGL line state */
  void restoreLineState() const;
};

}  // namespace mrpt::opengl
