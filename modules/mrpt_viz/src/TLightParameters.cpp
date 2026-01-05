/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/TLightParameters.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace std;

void TLightParameters::writeToStream(mrpt::serialization::CArchive& out) const
{
  const uint8_t version = 3;
  out << version;

  out << diffuse << ambient << specular << direction << color;
  out << shadow_bias << shadow_bias_cam2frag << shadow_bias_normal;               // v2
  out << eyeDistance2lightShadowExtension << minimum_shadow_map_extension_ratio;  // v3
}

void TLightParameters::readFromStream(mrpt::serialization::CArchive& in)
{
  uint8_t version;
  in >> version;

  switch (version)
  {
    case 0:
    {
      mrpt::img::TColorf diffuseCol, ambientCol, specularCol;
      in >> diffuseCol >> ambientCol >> specularCol >> direction;
      ambient = ambientCol.R;
      specular = specularCol.R;
      diffuse = 1.0f;
      color = diffuseCol;
    }
    break;
    case 1:
    case 2:
    case 3:
      in >> diffuse >> ambient >> specular >> direction >> color;
      if (version >= 2) in >> shadow_bias >> shadow_bias_cam2frag >> shadow_bias_normal;
      if (version >= 3)
        in >> eyeDistance2lightShadowExtension >> minimum_shadow_map_extension_ratio;
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
