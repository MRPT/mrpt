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
/** GPU-side proxy for rendering point clouds.
 *
 * This proxy handles rendering of any CVisualObject that uses the POINTS shader,
 * such as:
 * - CPointCloud
 * - CPointCloudColoured
 * - Any object derived from VisualObjectParams_Points
 *
 * The proxy manages:
 * - Vertex buffer (3D positions)
 * - Color buffer (per-vertex RGBA colors)
 * - Point rendering parameters (size, variable sizing)
 * - VAO for efficient attribute binding
 *
 * Point rendering features:
 * - Fixed point size or variable (distance-based) point size
 * - Per-vertex colors
 * - Alpha blending for transparency
 *
 * Typical data flow:
 * 1. compile(): Upload initial point positions and colors to GPU
 * 2. updateBuffers(): Re-upload when points change
 * 3. render(): Draw points every frame
 *
 * \sa PointsProxyBase, mrpt::viz::VisualObjectParams_Points
 * \ingroup mrpt_opengl_grp
 */
class PointsProxy : public PointsProxyBase
{
 public:
  PointsProxy() = default;
  ~PointsProxy() override = default;

  /** @name RenderableProxy Interface Implementation
   * @{ */

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  const char* typeName() const override { return "PointsProxy"; }

  /** @} */

 private:
  /** Cached point rendering parameters (to avoid re-reading on every render) */
  struct PointParams
  {
    float pointSize = 1.0f;
    bool variablePointSize = true;
    float variablePointSize_K = 0.1f;
    float variablePointSize_DepthScale = 0.1f;
  };

  /** Cached parameters from last compile/update */
  mutable PointParams m_params;

  /** Helper: Extract point rendering parameters from source object */
  void extractPointParams(const mrpt::viz::CVisualObject* sourceObj);

  /** Helper: Upload point-specific uniforms to shader */
  void uploadPointUniforms(const RenderContext& rc) const;
};

}  // namespace mrpt::opengl