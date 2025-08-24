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

#include "opengl-precomp.h"  // Precompiled header
//
#include <mrpt/opengl/CEllipsoidRangeBearing2D.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CEllipsoidRangeBearing2D, CRenderizableShaderWireFrame, mrpt::opengl)

/*---------------------------------------------------------------
              transformFromParameterSpace
  ---------------------------------------------------------------*/
void CEllipsoidRangeBearing2D::transformFromParameterSpace(
    const std::vector<BASE::array_parameter_t>& in_pts,
    std::vector<BASE::array_point_t>& out_pts) const
{
  MRPT_START

  // (range,bearing) --> (x,y)
  const size_t N = in_pts.size();
  out_pts.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    const float range = in_pts[i][0];
    const float bearing = in_pts[i][1];
    out_pts[i][0] = range * cosf(bearing);
    out_pts[i][1] = range * sinf(bearing);
  }

  MRPT_END
}

uint8_t CEllipsoidRangeBearing2D::serializeGetVersion() const { return 0; }
void CEllipsoidRangeBearing2D::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  BASE::thisclass_writeToStream(out);
}

void CEllipsoidRangeBearing2D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      readFromStreamRender(in);
      BASE::thisclass_readFromStream(in);
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CRenderizable::notifyChange();
}
