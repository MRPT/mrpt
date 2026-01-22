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

#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TRenderMatrices.h>

#include <map>

namespace mrpt::opengl
{
class CRenderizable;

/** Element in a render queue: a proxy plus its rendering state.
 * \ingroup mrpt_opengl_grp
 */
struct RenderQueueElement
{
  /** The object to render (non-owning pointer, owned by CompiledViewport) */
  class RenderableProxy* proxy = nullptr;

  /** Rendering state for this object */
  TRenderMatrices renderState;

  RenderQueueElement() = default;
  RenderQueueElement(RenderableProxy* p, const TRenderMatrices& state) :
      proxy(p), renderState(state)
  {
  }
};

/** A render queue: map from shader_id to sorted list of objects to render.
 * Objects are sorted by depth for correct transparency rendering.
 * \ingroup mrpt_opengl_grp
 */
using RenderQueue = std::map<shader_id_t, std::multimap<float, RenderQueueElement>>;

/** Stats for the rendering queue
 * \ingroup mrpt_opengl_grp
 */
struct RenderQueueStats
{
  RenderQueueStats() = default;

  size_t numObjTotal = 0, numObjRendered = 0;
};

/** Computes the eye-view depth of an object, and whether any part of its
 * bounding box is visible by the camera in the current state.
 * Return:
 *  - double: Depth of representative point.
 *  - bool: visible (at least in part)
 *  - bool: the whole bbox is visible (only checked for CSetOfObjects)
 * \ingroup mrpt_opengl_grp */
std::tuple<double, bool, bool> depthAndVisibleInView(
    const CRenderizable* obj, const mrpt::opengl::TRenderMatrices& objState, bool skipCullChecks);

}  // namespace mrpt::opengl
