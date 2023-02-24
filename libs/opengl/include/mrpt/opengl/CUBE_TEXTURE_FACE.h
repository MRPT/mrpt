/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

namespace mrpt::opengl
{
/** Enum type for each of the 6 faces of a Cube Texture.
 *
 *  Note that these enums must be defined in the same order than OpenGL API
 *  constants:
 *
 *  #define GL_TEXTURE_CUBE_MAP_POSITIVE_X    0x8515
 *  #define GL_TEXTURE_CUBE_MAP_NEGATIVE_X    0x8516
 *  #define GL_TEXTURE_CUBE_MAP_POSITIVE_Y    0x8517
 *  #define GL_TEXTURE_CUBE_MAP_NEGATIVE_Y    0x8518
 *  #define GL_TEXTURE_CUBE_MAP_POSITIVE_Z    0x8519
 *  #define GL_TEXTURE_CUBE_MAP_NEGATIVE_Z    0x851A
 *
 * \ingroup mrpt_opengl_grp
 */
enum class CUBE_TEXTURE_FACE
{
	RIGHT = 0,	// +X
	LEFT,  // -X
	TOP,  // +Y
	BOTTOM,	 // -Y
	BACK,  // +Z
	FRONT  // -Z
};

}  // namespace mrpt::opengl
