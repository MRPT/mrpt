/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/serialization/CArchive.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/system/os.h>
#include <mrpt/math/matrix_serialization.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRobotPose, CObservation, mrpt::obs)

uint8_t CObservationRobotPose::serializeGetVersion() const { return 0; }
void CObservationRobotPose::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << pose;
	out << sensorLabel << timestamp;
}

void CObservationRobotPose::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> pose;
			in >> sensorLabel >> timestamp;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationRobotPose::getSensorPose(CPose3D& out_sensorPose) const
{
	out_sensorPose = sensorPose;
}

void CObservationRobotPose::setSensorPose(const CPose3D& newSensorPose)
{
	sensorPose = newSensorPose;
}

void CObservationRobotPose::getDescriptionAsText(std::ostream& o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Sensor pose: " << sensorPose << endl;
	o << "Pose: " << pose.asString() << endl;
}
