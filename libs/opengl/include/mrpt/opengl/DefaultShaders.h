/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/Shader.h>
#include <cstdint>

namespace mrpt::opengl
{
// Note: do not use a enum-class to allow easy conversion from names to values:
// It's preferred to left the entities that are more likely to be transparent
// (triangles) at the end (higher IDs), so they get rendered last and
// transparency works better. Within each shader, objects are also sorted by
// inverse eye-distance for this reason.
struct DefaultShaderID
{
	static constexpr shader_id_t POINTS = 0;
	static constexpr shader_id_t WIREFRAME = 1;
	static constexpr shader_id_t TEXT = 2;
	static constexpr shader_id_t TRIANGLES = 3;
	static constexpr shader_id_t TEXTURED_TRIANGLES = 4;
};

/** Loads a set of OpenGL Vertex+Fragment shaders from the default library
 * available in mrpt::opengl.
 *
 * \sa CRenderizable
 * \ingroup mrpt_opengl_grp
 */
Program::Ptr LoadDefaultShader(const shader_id_t id);

}  // namespace mrpt::opengl
