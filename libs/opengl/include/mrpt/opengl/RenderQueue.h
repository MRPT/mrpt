/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/Shader.h>
#include <mrpt/opengl/TRenderMatrices.h>
#include <cstdint>
#include <deque>
#include <map>

namespace mrpt::opengl
{
class CRenderizable;

/** Each element in the queue to be rendered for each keyframe
 * \ingroup mrpt_opengl_grp
 */
struct RenderQueueElement
{
	RenderQueueElement() = default;

	RenderQueueElement(
		const mrpt::opengl::CRenderizable* obj,
		const mrpt::opengl::TRenderMatrices& state)
		: object(obj), renderState(state)
	{
	}

	const mrpt::opengl::CRenderizable* object = nullptr;
	mrpt::opengl::TRenderMatrices renderState = {};
};

/** A queue for rendering, sorted by shader program to minimize changes of
 * OpenGL shader programs while rendering a scene.
 * Filled by sortRenderObjectsByShader()
 * \ingroup mrpt_opengl_grp
 */
using RenderQueue = std::map<shader_id_t, std::deque<RenderQueueElement>>;

}  // namespace mrpt::opengl
