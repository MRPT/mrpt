/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/img/TColor.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>

namespace mrpt::opengl
{
/** Unidirectional lighting model parameters for triangle shaders.
 * Refer to standard OpenGL literature and tutorials for the meaning of each
 * field, and to the shader GLSL code itself.
 * \ingroup mrpt_opengl_grp
 */
struct TLightParameters
{
  TLightParameters() = default;
  ~TLightParameters() = default;

  mrpt::img::TColorf color = {1.0f, 1.0f, 1.0f};

  float diffuse = 0.8f;
  float ambient = 0.2f;
  float specular = 0.95f;

  /** Light direction (must be normalized) */
  mrpt::math::TVector3Df direction = {-0.40825f, -0.40825f, -0.81650f};

  /** Shadow tuning parameters ("anti shadow acne") */
  float shadow_bias = 1e-5, shadow_bias_cam2frag = 1e-5, shadow_bias_normal = 1e-4;

  /** Multiplier from eye distance to the length size of the squared area in
   * which to evaluate shadow casting by unidirectional light.
   * Unitless (meter/meter).
   * \note (New in MRPT 2.10.0)
   */
  double eyeDistance2lightShadowExtension = 2.0;

  /** Minimum extension (in [0,1] ratio of the light distance) of the shadow
   * map square ortho frustum. Should be roughly the maximum area of the
   * largest room for indoor environments to ensure no missing shadows in
   * distant areas.
   *
   * \note (New in MRPT 2.10.0)
   */
  float minimum_shadow_map_extension_ratio = 0.03f;

  void writeToStream(mrpt::serialization::CArchive& out) const;
  void readFromStream(mrpt::serialization::CArchive& in);

  DECLARE_TTYPENAME_CLASSNAME(mrpt::opengl::TLightParameters)
};

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, mrpt::opengl::TLightParameters& o);
mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const mrpt::opengl::TLightParameters& o);

}  // namespace mrpt::opengl
