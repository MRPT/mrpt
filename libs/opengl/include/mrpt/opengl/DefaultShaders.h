/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
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
  // Regular geometric elements:
  static constexpr shader_id_t POINTS = 0;
  static constexpr shader_id_t WIREFRAME = 1;
  static constexpr shader_id_t TEXT = 2;
  static constexpr shader_id_t TRIANGLES_LIGHT = 3;
  static constexpr shader_id_t TEXTURED_TRIANGLES_LIGHT = 4;
  static constexpr shader_id_t TRIANGLES_NO_LIGHT = 5;
  static constexpr shader_id_t TEXTURED_TRIANGLES_NO_LIGHT = 6;

  // Shadow generation 1st/2nd pass shaders:
  static constexpr shader_id_t TRIANGLES_SHADOW_1ST = 10;
  static constexpr shader_id_t TRIANGLES_SHADOW_2ND = 11;
  static constexpr shader_id_t TEXTURED_TRIANGLES_SHADOW_1ST = 12;
  static constexpr shader_id_t TEXTURED_TRIANGLES_SHADOW_2ND = 13;

  // Special effects:
  static constexpr shader_id_t SKYBOX = 20;
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
