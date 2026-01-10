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
  // Regular geometric elements:
  static constexpr shader_id_t POINTS = 0;
  static constexpr shader_id_t WIREFRAME = 1;
  static constexpr shader_id_t TEXT = 2;
  static constexpr shader_id_t TRIANGLES_LIGHT = 10;
  static constexpr shader_id_t TEXTURED_TRIANGLES_LIGHT = 11;
  static constexpr shader_id_t TRIANGLES_NO_LIGHT = 12;
  static constexpr shader_id_t TEXTURED_TRIANGLES_NO_LIGHT = 13;

  // Shadow generation 1st/2nd pass shaders:
  static constexpr shader_id_t TRIANGLES_SHADOW_1ST = 20;
  static constexpr shader_id_t TRIANGLES_SHADOW_2ND = 21;
  static constexpr shader_id_t TEXTURED_TRIANGLES_SHADOW_1ST = 22;
  static constexpr shader_id_t TEXTURED_TRIANGLES_SHADOW_2ND = 23;

  // Special effects:
  static constexpr shader_id_t SKYBOX = 5;  // render *before* potentially transparent triangles
  static constexpr shader_id_t DEBUG_TEXTURE_TO_SCREEN = 30;
  static constexpr shader_id_t NONE = 31;  //!< Skip rendering
};

/** Loads a set of OpenGL Vertex+Fragment shaders from the default library
 * available in mrpt::opengl.
 *
 * \sa CRenderizable
 * \ingroup mrpt_opengl_grp
 */
Program::Ptr LoadDefaultShader(const shader_id_t id);

// Use GL_TEXTURE0 for diffuse map material textures:
static constexpr int MATERIAL_DIFFUSE_TEXTURE_UNIT = 0;

// Use GL_TEXTURE1 for shadow map:
static constexpr int SHADOW_MAP_TEXTURE_UNIT = 1;

}  // namespace mrpt::opengl
