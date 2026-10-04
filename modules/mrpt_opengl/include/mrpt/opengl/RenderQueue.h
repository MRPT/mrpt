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

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/math/TBoundingBox.h>
#include <mrpt/opengl/Shader.h>

#include <vector>

namespace mrpt::opengl
{
// Forward declarations
class RenderableProxy;

/** Element in a render queue: a proxy, the shader to draw it with, and its
 * per-object matrices.
 * \ingroup mrpt_opengl_grp
 */
struct RenderQueueElement
{
  /** The object to render (non-owning pointer, owned by CompiledViewport) */
  RenderableProxy* proxy = nullptr;

  shader_id_t shader = 0;

  /** Eye-space depth of the object (larger is farther), for sorting */
  float depth = 0;

  /** Model, view-model and projection-view-model matrices of the object */
  mrpt::math::CMatrixFloat44 m_matrix;
  mrpt::math::CMatrixFloat44 mv_matrix;
  mrpt::math::CMatrixFloat44 pmv_matrix;
};

/** The objects to render in one pass, in three layers drawn in this order:
 * opaque objects (grouped by shader, then front to back to make the most of
 * early depth rejection), background objects (e.g. sky boxes), and finally
 * transparent objects, from back to front so they blend correctly.
 * \ingroup mrpt_opengl_grp
 */
struct RenderQueue
{
  std::vector<RenderQueueElement> opaque;
  std::vector<RenderQueueElement> background;
  std::vector<RenderQueueElement> transparent;

  void clear()
  {
    opaque.clear();
    background.clear();
    transparent.clear();
  }

  [[nodiscard]] bool empty() const
  {
    return opaque.empty() && background.empty() && transparent.empty();
  }

  /** Sorts each layer into its drawing order. */
  void sort();
};

/** Conservative culling test: returns false only if the local axis-aligned
 * box `bb`, transformed by `clipFromLocal` (e.g. P*V*M), lies entirely
 * outside one of the clip volume planes. Valid for perspective and
 * orthographic projections alike. Bits of `planes` select which planes to
 * test: 1=left, 2=right, 4=bottom, 8=top, 16=near, 32=far.
 * \ingroup mrpt_opengl_grp
 */
[[nodiscard]] bool boxIntersectsClipVolume(
    const mrpt::math::TBoundingBoxf& bb,
    const mrpt::math::CMatrixFloat44& clipFromLocal,
    unsigned int planes = 0x3F);

}  // namespace mrpt::opengl
