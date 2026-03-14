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

#include <mrpt/img/TColor.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/serialization/serialization_frwds.h>
#include <mrpt/typemeta/TTypeName.h>

#include <vector>

namespace mrpt::viz
{
/** Maximum number of simultaneous lights supported by the shader pipeline. */
static constexpr int MAX_LIGHTS = 8;

/** Light source type.
 * \ingroup mrpt_viz_grp
 */
enum class TLightType : uint8_t
{
  Directional = 0,  ///< Infinitely far away, parallel rays (e.g. the sun)
  Point = 1,        ///< Emits in all directions from a position
  Spot = 2          ///< Emits in a cone from a position along a direction
};

/** A single light source (directional, point, or spot).
 * \ingroup mrpt_viz_grp
 */
struct TLight
{
  TLightType type = TLightType::Directional;

  /** Light color */
  mrpt::img::TColorf color = {1.0f, 1.0f, 1.0f};

  /** Diffuse intensity [0,1] */
  float diffuse = 0.8f;

  /** Specular intensity [0,1] */
  float specular = 0.95f;

  /** Direction vector (must be normalized). Used by Directional and Spot. */
  mrpt::math::TVector3Df direction = {-0.40825f, -0.40825f, -0.81650f};

  /** Position in world coordinates. Used by Point and Spot. */
  mrpt::math::TPoint3Df position = {0, 0, 0};

  /** Attenuation factors for Point/Spot: intensity = 1 / (constant +
   *  linear*d + quadratic*d²). Ignored for Directional. */
  float attenuation_constant = 1.0f;
  float attenuation_linear = 0.09f;
  float attenuation_quadratic = 0.032f;

  /** Spot light inner cutoff angle in degrees (full intensity cone). */
  float spot_inner_cutoff_deg = 12.5f;
  /** Spot light outer cutoff angle in degrees (light fades to zero). */
  float spot_outer_cutoff_deg = 17.5f;

  /** Factory: creates a directional light. */
  static TLight Directional(
      const mrpt::math::TVector3Df& dir,
      const mrpt::img::TColorf& color = {1.0f, 1.0f, 1.0f},
      float diffuse = 0.8f,
      float specular = 0.95f)
  {
    TLight l;
    l.type = TLightType::Directional;
    l.direction = dir;
    l.color = color;
    l.diffuse = diffuse;
    l.specular = specular;
    return l;
  }

  /** Factory: creates a point light. */
  static TLight PointLight(
      const mrpt::math::TPoint3Df& pos,
      const mrpt::img::TColorf& color = {1.0f, 1.0f, 1.0f},
      float diffuse = 0.8f,
      float specular = 0.95f,
      float att_constant = 1.0f,
      float att_linear = 0.09f,
      float att_quadratic = 0.032f)
  {
    TLight l;
    l.type = TLightType::Point;
    l.position = pos;
    l.color = color;
    l.diffuse = diffuse;
    l.specular = specular;
    l.attenuation_constant = att_constant;
    l.attenuation_linear = att_linear;
    l.attenuation_quadratic = att_quadratic;
    return l;
  }

  /** Factory: creates a spot light. */
  static TLight SpotLight(
      const mrpt::math::TPoint3Df& pos,
      const mrpt::math::TVector3Df& dir,
      float innerCutoffDeg = 12.5f,
      float outerCutoffDeg = 17.5f,
      const mrpt::img::TColorf& color = {1.0f, 1.0f, 1.0f},
      float diffuse = 0.8f,
      float specular = 0.95f,
      float att_constant = 1.0f,
      float att_linear = 0.09f,
      float att_quadratic = 0.032f)
  {
    TLight l;
    l.type = TLightType::Spot;
    l.position = pos;
    l.direction = dir;
    l.color = color;
    l.diffuse = diffuse;
    l.specular = specular;
    l.attenuation_constant = att_constant;
    l.attenuation_linear = att_linear;
    l.attenuation_quadratic = att_quadratic;
    l.spot_inner_cutoff_deg = innerCutoffDeg;
    l.spot_outer_cutoff_deg = outerCutoffDeg;
    return l;
  }

  void writeToStream(mrpt::serialization::CArchive& out) const;
  void readFromStream(mrpt::serialization::CArchive& in);
};

/** Lighting model parameters for a viewport.
 *
 * Contains an array of light sources (up to MAX_LIGHTS) plus global
 * settings (ambient, shadow tuning, gamma correction).
 *
 * Shadow mapping is only performed for the first directional light in the
 * array (the "primary" directional light).
 *
 * \sa TLight, Viewport
 * \ingroup mrpt_viz_grp
 */
struct TLightParameters
{
  TLightParameters()
  {
    // Default: one directional light matching the legacy default
    lights.push_back(TLight::Directional({-0.40825f, -0.40825f, -0.81650f}));
  }
  ~TLightParameters() = default;

  /** The individual light sources (up to MAX_LIGHTS). */
  std::vector<TLight> lights;

  /** Global ambient intensity [0,1]. Applied uniformly regardless of
   *  light sources. */
  float ambient = 0.2f;

  /** Shadow tuning parameters ("anti shadow acne").
   *  Applied to the primary directional light only. */
  float shadow_bias = 1e-5f;
  float shadow_bias_cam2frag = 1e-5f;
  float shadow_bias_normal = 1e-4f;

  /** Multiplier from eye distance to the length size of the squared area in
   * which to evaluate shadow casting by the primary directional light.
   * Unitless (meter/meter).
   */
  double eyeDistance2lightShadowExtension = 2.0;

  /** Minimum extension (in [0,1] ratio of the light distance) of the shadow
   * map square ortho frustum. */
  float minimum_shadow_map_extension_ratio = 0.03f;

  /** If true (default), enables physically-correct gamma correction via the
   * GPU sRGB pipeline. */
  bool gamma_correction = true;

  /** Returns the direction of the first directional light, or a fallback
   *  if none exists. Used for shadow mapping. */
  [[nodiscard]] mrpt::math::TVector3Df primaryDirectionalDirection() const
  {
    for (const auto& l : lights)
    {
      if (l.type == TLightType::Directional)
      {
        return l.direction;
      }
    }
    return {-0.40825f, -0.40825f, -0.81650f};
  }

  void writeToStream(mrpt::serialization::CArchive& out) const;
  void readFromStream(mrpt::serialization::CArchive& in);

  DECLARE_TTYPENAME_CLASSNAME(mrpt::viz::TLightParameters)
};

mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, mrpt::viz::TLightParameters& o);
mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const mrpt::viz::TLightParameters& o);

}  // namespace mrpt::viz
