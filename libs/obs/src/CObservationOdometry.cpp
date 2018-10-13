/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers

#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/os.h>

using namespace mrpt::obs;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationOdometry, CObservation, mrpt::obs)

/** Constructor
 */
CObservationOdometry::CObservationOdometry()
	: odometry(),

	  velocityLocal(.0, .0, .0)
{
}

uint8_t CObservationOdometry::serializeGetVersion() const { return 2; }
void CObservationOdometry::serializeTo(mrpt::serialization::CArchive& out) const
{
	// The data
	out << odometry << sensorLabel
		<< timestamp
		// Added in V1:
		<< hasEncodersInfo;
	if (hasEncodersInfo) out << encoderLeftTicks << encoderRightTicks;
	out << hasVelocities;
	if (hasVelocities) out << velocityLocal;
}

void CObservationOdometry::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	MRPT_UNUSED_PARAM(in);
	switch (version)
	{
		case 0:
		case 1:
		case 2:
		{
			in >> odometry >> sensorLabel >> timestamp;

			if (version >= 1)
			{
				in >> hasEncodersInfo;
				if (hasEncodersInfo || version < 2)
					in >> encoderLeftTicks >> encoderRightTicks;

				in >> hasVelocities;
				if (version < 2)
				{
					float vx, w;
					in >> vx >> w;
					velocityLocal.vx = vx;
					velocityLocal.vy = .0;
					velocityLocal.omega = w;
				}
				else
				{  // v2
					if (hasVelocities) in >> velocityLocal;
				}
			}
			else
			{
				hasEncodersInfo = false;
				encoderLeftTicks = encoderRightTicks = 0;
				hasVelocities = false;
			}
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CObservationOdometry::getDescriptionAsText(std::ostream& o) const
{
	CObservation::getDescriptionAsText(o);

	o << std::endl << "Odometry reading: " << odometry << std::endl;

	// Additional data:
	if (hasEncodersInfo)
	{
		o << format(
			" Encoder info: deltaL=%i deltaR=%i\n", encoderLeftTicks,
			encoderRightTicks);
	}
	else
		o << "Encoder info: Not available!\n";

	if (hasVelocities)
	{
		o << format("Velocity info: %s\n", velocityLocal.asString().c_str());
	}
	else
		o << "Velocity info: Not available!\n";
}
