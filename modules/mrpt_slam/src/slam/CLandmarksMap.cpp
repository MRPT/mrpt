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

#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/serialization/CArchive.h>

IMPLEMENTS_SERIALIZABLE(CLandmarksMap, CMetricMap, mrpt::maps)

uint8_t mrpt::maps::CLandmarksMap::serializeGetVersion() const { return 0; }

void mrpt::maps::CLandmarksMap::serializeTo(mrpt::serialization::CArchive& out) const
{
  const auto n = static_cast<uint32_t>(landmarks.size());
  out << n;
  for (uint32_t i = 0; i < n; i++)
  {
    const auto& lm = *landmarks.get(i);
    out << lm.ID;
    out << lm.pose_mean.x << lm.pose_mean.y << lm.pose_mean.z;
    out << lm.pose_cov_11 << lm.pose_cov_22 << lm.pose_cov_33;
    out << lm.pose_cov_12 << lm.pose_cov_13 << lm.pose_cov_23;
    out << static_cast<uint32_t>(lm.seenTimesCount);
  }
}

void mrpt::maps::CLandmarksMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      uint32_t n;
      in >> n;
      landmarks.clear();
      for (uint32_t i = 0; i < n; i++)
      {
        CLandmark lm;
        in >> lm.ID;
        in >> lm.pose_mean.x >> lm.pose_mean.y >> lm.pose_mean.z;
        in >> lm.pose_cov_11 >> lm.pose_cov_22 >> lm.pose_cov_33;
        in >> lm.pose_cov_12 >> lm.pose_cov_13 >> lm.pose_cov_23;
        uint32_t cnt;
        in >> cnt;
        lm.seenTimesCount = cnt;
        landmarks.push_back(lm);
      }
      break;
    }
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}
