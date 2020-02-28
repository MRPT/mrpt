/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

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
		"  (as time_t): %.09f\n",
		mrpt::system::dateTimeToString(timestamp).c_str(),
		mrpt::Clock::toDouble(timestamp));

	o << "  (as TTimestamp): " << timestamp
	  << "\n"
		 "Sensor label: '"
	  << sensorLabel << "'"
	  << "\n\n";
}

std::string CObservation::getDescriptionAsTextValue() const
{
	std::stringstream ss;
	getDescriptionAsText(ss);
	return ss.str();
}
