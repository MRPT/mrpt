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

#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

#include <iomanip>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;

IMPLEMENTS_VIRTUAL_SERIALIZABLE(CObservation, CSerializable, mrpt::obs)

void CObservation::getSensorPose(mrpt::math::TPose3D& out_sensorPose) const
{
  CPose3D p;
  getSensorPose(p);
  out_sensorPose = p.asTPose();
}

void CObservation::setSensorPose(const mrpt::math::TPose3D& newSensorPose)
{
  setSensorPose(CPose3D(newSensorPose));
}

void CObservation::swap(CObservation& o)
{
  std::swap(timestamp, o.timestamp);
  std::swap(sensorLabel, o.sensorLabel);
}

void CObservation::getDescriptionAsText(std::ostream& o) const
{
  using namespace mrpt::system;  // for the TTimeStamp << op

  o << mrpt::format(
      "Timestamp (UTC): %s\n"
      "        (local): %s\n"
      "    (as time_t): %.09f\n",
      mrpt::system::dateTimeToString(timestamp).c_str(),
      mrpt::system::dateTimeLocalToString(timestamp).c_str(), mrpt::Clock::toDouble(timestamp));

  o << "  (as TTimestamp): " << timestamp
    << "\n"
       "Sensor label: '"
    << sensorLabel << "'"
    << "\n\n";

  o << "ClassName: " << this->GetRuntimeClass()->className << "\n"
    << "\n";
}

std::string CObservation::asString() const
{
  std::stringstream ss;
  getDescriptionAsText(ss);
  return ss.str();
}
