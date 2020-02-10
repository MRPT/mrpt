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
#include <cstdint>

namespace mrpt::opengl
{
/** Type for IDs of shaders.
 * \sa
 * \ingroup mrpt_opengl_grp
 */
using shader_id_t = uint8_t;

// Note: do not use a enum-class to allow easy conversion from names to values:
struct DefaultShaderID
{
	static constexpr shader_id_t POINTS = 0;
	static constexpr shader_id_t WIREFRAME = 1;
	static constexpr shader_id_t TRIANGLES = 2;
	static constexpr shader_id_t TEXTURED_TRIANGLES = 3;
	static constexpr shader_id_t TEXT = 4;
};

/** Loads a set of OpenGL Vertex+Fragment shaders from the default library
 * available in mrpt::opengl.
 *
 * \sa CRenderizable
 * \ingroup mrpt_opengl_grp
 */
Program::Ptr LoadDefaultShader(const shader_id_t id);

}  // namespace mrpt::opengl
