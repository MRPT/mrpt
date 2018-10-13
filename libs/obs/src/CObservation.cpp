/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservation.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>
#include <mrpt/math/lightweight_geom_data.h>

#include <mrpt/poses/CPose3D.h>
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

	o << "Timestamp (UTC): " << mrpt::system::dateTimeToString(timestamp)
	  << std::endl;
	o << "  (as time_t): " << std::fixed << std::setprecision(5)
	  << mrpt::system::timestampTotime_t(timestamp) << std::endl;
	o << "  (as TTimestamp): " << timestamp << std::endl;
	o << "Sensor label: '" << sensorLabel << "'" << std::endl;
	o << std::endl;
}
