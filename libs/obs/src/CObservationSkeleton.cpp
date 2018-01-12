/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservationSkeleton.h>
#include <mrpt/serialization/CArchive.h>
#include <iostream>

using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace std;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationSkeleton, CObservation, mrpt::obs)

// Helpful macros for reading and writing joints to a stream
#define WRITE_JOINT(_J) out << _J.x << _J.y << _J.z << _J.conf;
#define READ_JOINT(_J) in >> _J.x >> _J.y >> _J.z >> _J.conf;

uint8_t CObservationSkeleton::serializeGetVersion() const { return 2; }
void CObservationSkeleton::serializeTo(mrpt::serialization::CArchive& out) const
{
	WRITE_JOINT(head)
	WRITE_JOINT(neck)
	WRITE_JOINT(torso)

	WRITE_JOINT(left_shoulder)
	WRITE_JOINT(left_elbow)
	WRITE_JOINT(left_hand)
	WRITE_JOINT(left_hip)
	WRITE_JOINT(left_knee)
	WRITE_JOINT(left_foot)

	WRITE_JOINT(right_shoulder)
	WRITE_JOINT(right_elbow)
	WRITE_JOINT(right_hand)
	WRITE_JOINT(right_hip)
	WRITE_JOINT(right_knee)
	WRITE_JOINT(right_foot)

	out << sensorLabel << timestamp << sensorPose;
}

void CObservationSkeleton::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			READ_JOINT(head)
			READ_JOINT(neck)
			READ_JOINT(torso)

			READ_JOINT(left_shoulder)
			READ_JOINT(left_elbow)
			READ_JOINT(left_hand)
			READ_JOINT(left_hip)
			READ_JOINT(left_knee)
			READ_JOINT(left_foot)

			READ_JOINT(right_shoulder)
			READ_JOINT(right_elbow)
			READ_JOINT(right_hand)
			READ_JOINT(right_hip)
			READ_JOINT(right_knee)
			READ_JOINT(right_foot)

			in >> sensorLabel;
			in >> timestamp;
			if (version >= 2)
			{
				in >> sensorPose;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationSkeleton::getDescriptionAsText(std::ostream& o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Sensor pose on the robot: " << sensorPose << endl;

	// ----------------------------------------------------------------------
	//              CObservationSkeleton
	// ----------------------------------------------------------------------
	o << endl << "Joint Positions (x, y, z) [mm] -- confidence" << endl;

#define PRINT_JOINT(_J)                                                       \
	cout << "\t" << #_J << ":\t(" << this->_J.x << ", " << this->_J.y << ", " \
		 << this->_J.z << ") -- " << this->_J.conf << endl;

	PRINT_JOINT(head)
	PRINT_JOINT(neck)
	PRINT_JOINT(torso)

	PRINT_JOINT(left_shoulder)
	PRINT_JOINT(left_elbow)
	PRINT_JOINT(left_hand)
	PRINT_JOINT(left_hip)
	PRINT_JOINT(left_knee)
	PRINT_JOINT(left_foot)

	PRINT_JOINT(right_shoulder)
	PRINT_JOINT(right_elbow)
	PRINT_JOINT(right_hand)
	PRINT_JOINT(right_hip)
	PRINT_JOINT(right_knee)
	PRINT_JOINT(right_foot)
}
