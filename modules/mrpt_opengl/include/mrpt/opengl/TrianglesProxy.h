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
#include <mrpt/viz/CVisualObject.h>

namespace mrpt::opengl
{
/** GPU-side proxy for rendering triangle meshes.
 *
 * This proxy handles rendering of any CVisualObject that uses triangle-based
 * shaders, such as:
 * - CMesh
 * - CBox
 * - CSphere
 * - CCylinder
 * - Any object derived from VisualObjectParams_Triangles
 *
 * The proxy manages:
 * - Vertex buffer (3D triangle vertices)
 * - Normal buffer (per-vertex normals for lighting)
 * - Color buffer (per-vertex RGBA colors)
 * - VAO for efficient attribute binding
 * - Lighting and material parameters
 * - Face culling modes
 *
 * Triangle rendering features:
 * - Phong lighting (ambient, diffuse, specular)
 * - Per-vertex colors with optional lighting modulation
 * - Face culling (FRONT, BACK, NONE)
 * - Shadow casting (2-pass rendering)
 * - Material properties (shininess)
 *
 * Shader selection:
 * - Normal rendering: TRIANGLES_LIGHT or TRIANGLES_NO_LIGHT
 * - Shadow 1st pass: TRIANGLES_SHADOW_1ST (depth only)
 * - Shadow 2nd pass: TRIANGLES_SHADOW_2ND (with shadow mapping)
 *
 * Typical data flow:
 * 1. compile(): Upload triangle vertices, normals, colors to GPU
 * 2. updateBuffers(): Re-upload when mesh changes
 * 3. render(): Draw triangles with appropriate shader every frame
 *
 * \sa TrianglesProxyBase, mrpt::viz::VisualObjectParams_Triangles
 * \ingroup mrpt_opengl_grp
 */
class TrianglesProxy : public TrianglesProxyBase
{
 public:
  TrianglesProxy() = default;
  ~TrianglesProxy() override = default;

  /** @name RenderableProxy Interface Implementation
   * @{ */

  void compile(const mrpt::viz::CVisualObject* sourceObj) override;
  void updateBuffers(const mrpt::viz::CVisualObject* sourceObj) override;
  void render(const RenderContext& rc) const override;

  std::vector<shader_id_t> requiredShaders() const override;

  const char* typeName() const override { return "TrianglesProxy"; }

  /** @} */

 private:
  /** Cached triangle rendering parameters */
  struct TriangleParams
  {
    bool lightEnabled = true;
    mrpt::viz::TCullFace cullFace = mrpt::viz::TCullFace::NONE;
    float materialShininess = 0.2f;
  };

  /** Cached parameters from last compile/update */
  mutable TriangleParams m_params;

  /** Helper: Extract triangle rendering parameters from source object */
  void extractTriangleParams(const mrpt::viz::CVisualObject* sourceObj);

  /** Helper: Upload triangle-specific uniforms to shader */
  void uploadTriangleUniforms(const RenderContext& rc) const;

  /** Helper: Setup face culling based on m_params.cullFace */
  void setupFaceCulling() const;

  /** Helper: Restore face culling state */
  void restoreFaceCulling() const;

  /** Helper: Converts TTriangle array to flat vertex/normal/color arrays */
  void extractTriangleData(
      const std::vector<mrpt::viz::TTriangle>& triangles,
      std::vector<mrpt::math::TPoint3Df>& vertices,
      std::vector<mrpt::math::TVector3Df>& normals,
      std::vector<mrpt::img::TColor>& colors) const;
};

}  // namespace mrpt::opengl