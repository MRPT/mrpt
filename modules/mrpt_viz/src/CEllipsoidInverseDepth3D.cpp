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
#include <mrpt/viz/CEllipsoidInverseDepth3D.h>

using namespace mrpt;
using namespace mrpt::viz;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CEllipsoidInverseDepth3D, CVisualObject, mrpt::viz)

/*---------------------------------------------------------------
              transformFromParameterSpace
  ---------------------------------------------------------------*/
void CEllipsoidInverseDepth3D::transformFromParameterSpace(
    const std::vector<BASE::array_parameter_t>& in_pts,
    std::vector<BASE::array_point_t>& out_pts) const
{
  MRPT_START

  // (inv_range,yaw,pitch) --> (x,y,z)
  const size_t N = in_pts.size();
  out_pts.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    const float inv_range = in_pts[i][0];
    const float yaw = in_pts[i][1];
    const float pitch = in_pts[i][2];

    const float range =
        inv_range < 0 ? m_underflowMaxRange : (inv_range != 0 ? 1.f / inv_range : 0);

    out_pts[i][0] = range * cosf(yaw) * cosf(pitch);
    out_pts[i][1] = range * sinf(yaw) * cosf(pitch);
    out_pts[i][2] = -range * sinf(pitch);
  }

  MRPT_END
}

uint8_t CEllipsoidInverseDepth3D::serializeGetVersion() const { return 0; }
void CEllipsoidInverseDepth3D::serializeTo(mrpt::serialization::CArchive& out) const
{
  writeToStreamRender(out);
  BASE::thisclass_writeToStream(out);

  out << m_underflowMaxRange;
}

void CEllipsoidInverseDepth3D::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      readFromStreamRender(in);
      BASE::thisclass_readFromStream(in);

      in >> m_underflowMaxRange;
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
  CVisualObject::notifyChange();
}
