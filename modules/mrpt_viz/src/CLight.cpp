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
#include <mrpt/viz/CLight.h>

using namespace mrpt::viz;

IMPLEMENTS_SERIALIZABLE(CLight, CVisualObject, mrpt::viz)

uint8_t CLight::serializeGetVersion() const { return 0; }

void CLight::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  light().writeToStream(out);
}

void CLight::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      readFromStreamRender(in);
      TLight l;
      l.readFromStream(in);
      light(l);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

auto CLight::internalBoundingBoxLocal() const -> mrpt::math::TBoundingBoxf
{
  const auto l = light();
  if (l.type == TLightType::Directional)
  {
    return {};
  }
  return {l.position, l.position};
}
