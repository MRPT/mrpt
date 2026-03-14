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

#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/TLightParameters.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace std;

// ===== TLight serialization =====

void TLight::writeToStream(mrpt::serialization::CArchive& out) const
{
  const uint8_t version = 0;
  out << version;
  out << static_cast<uint8_t>(type);
  out << color << diffuse << specular << direction << position;
  out << attenuation_constant << attenuation_linear << attenuation_quadratic;
  out << spot_inner_cutoff_deg << spot_outer_cutoff_deg;
}

void TLight::readFromStream(mrpt::serialization::CArchive& in)
{
  uint8_t version;
  in >> version;

  switch (version)
  {
    case 0:
    {
      uint8_t t;
      in >> t;
      type = static_cast<TLightType>(t);
      in >> color >> diffuse >> specular >> direction >> position;
      in >> attenuation_constant >> attenuation_linear >> attenuation_quadratic;
      in >> spot_inner_cutoff_deg >> spot_outer_cutoff_deg;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

// ===== TLightParameters serialization =====

void TLightParameters::writeToStream(mrpt::serialization::CArchive& out) const
{
  const uint8_t version = 6;
  out << version;

  // v5: multi-light format, v6: + hemisphere ambient
  out << ambient;
  out << shadow_bias << shadow_bias_cam2frag << shadow_bias_normal;
  out << eyeDistance2lightShadowExtension << minimum_shadow_map_extension_ratio;
  out << gamma_correction;
  out << static_cast<uint8_t>(lights.size());
  for (const auto& l : lights) l.writeToStream(out);
  // v6:
  out << ambientSkyColor << ambientGroundColor;
}

void TLightParameters::readFromStream(mrpt::serialization::CArchive& in)
{
  uint8_t version;
  in >> version;

  switch (version)
  {
    case 0:
    {
      // Legacy v0: diffuse/ambient/specular were TColorf, direction was TVector3Df
      mrpt::img::TColorf diffuseCol, ambientCol, specularCol;
      mrpt::math::TVector3Df dir;
      in >> diffuseCol >> ambientCol >> specularCol >> dir;
      ambient = ambientCol.R;
      lights.clear();
      auto l = TLight::Directional(dir, diffuseCol, 1.0f, specularCol.R);
      lights.push_back(l);
    }
    break;
    case 1:
    case 2:
    case 3:
    case 4:
    {
      // Legacy single-light format
      float diffuse, specular;
      mrpt::math::TVector3Df direction;
      mrpt::img::TColorf color;
      in >> diffuse >> ambient >> specular >> direction >> color;
      if (version >= 2) in >> shadow_bias >> shadow_bias_cam2frag >> shadow_bias_normal;
      if (version >= 3)
        in >> eyeDistance2lightShadowExtension >> minimum_shadow_map_extension_ratio;
      if (version >= 4)
        in >> gamma_correction;
      else
        gamma_correction = true;

      lights.clear();
      auto l = TLight::Directional(direction, color, diffuse, specular);
      lights.push_back(l);
    }
    break;
    case 5:
    case 6:
    {
      in >> ambient;
      in >> shadow_bias >> shadow_bias_cam2frag >> shadow_bias_normal;
      in >> eyeDistance2lightShadowExtension >> minimum_shadow_map_extension_ratio;
      in >> gamma_correction;
      uint8_t numLights;
      in >> numLights;
      lights.resize(numLights);
      for (auto& l : lights) l.readFromStream(in);
      if (version >= 6)
        in >> ambientSkyColor >> ambientGroundColor;
      else
      {
        ambientSkyColor = {1.0f, 1.0f, 1.0f};
        ambientGroundColor = {1.0f, 1.0f, 1.0f};
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

namespace mrpt::viz
{
mrpt::serialization::CArchive& operator>>(
    mrpt::serialization::CArchive& in, mrpt::viz::TLightParameters& o)
{
  o.readFromStream(in);
  return in;
}
mrpt::serialization::CArchive& operator<<(
    mrpt::serialization::CArchive& out, const mrpt::viz::TLightParameters& o)
{
  o.writeToStream(out);
  return out;
}
}  // namespace mrpt::viz
