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

namespace mrpt::viz
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
 * \ingroup mrpt_viz_grp
 */
enum class CUBE_TEXTURE_FACE
{
  LEFT = 0,  // +X
  RIGHT,     // -X
  TOP,       // +Y
  BOTTOM,    // -Y
  FRONT,     // +Z
  BACK       // -Z
};

}  // namespace mrpt::viz
