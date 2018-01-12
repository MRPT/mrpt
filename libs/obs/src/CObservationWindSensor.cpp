/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservationWindSensor.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationWindSensor, CObservation, mrpt::obs)

uint8_t CObservationWindSensor::serializeGetVersion() const { return 3; }
void CObservationWindSensor::serializeTo(
	mrpt::serialization::CArchive& out) const
{
	out << speed << direction << sensorLabel << timestamp << sensorPoseOnRobot;
}

void CObservationWindSensor::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			in >> speed >> direction;
			if (version >= 1)
				in >> sensorLabel;
			else
				sensorLabel = "";

			if (version >= 2)
				in >> timestamp;
			else
				timestamp = INVALID_TIMESTAMP;

			if (version >= 3)
				in >> sensorPoseOnRobot;
			else
				sensorPoseOnRobot = CPose3D();
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationWindSensor::getSensorPose(CPose3D& out_sensorPose) const
{
	out_sensorPose = sensorPoseOnRobot;
}

void CObservationWindSensor::setSensorPose(const CPose3D& newSensorPose)
{
	sensorPoseOnRobot = newSensorPose;
}

void CObservationWindSensor::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);
}
