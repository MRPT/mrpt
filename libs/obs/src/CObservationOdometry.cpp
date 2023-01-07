/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precompiled headers
//
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
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
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

std::string CObservationOdometry::exportTxtHeader() const
{
	return mrpt::format(
		"%18s %18s %18s "  // GLOBAL_ODO_{x,y,phi}
		"%18s %18s %18s "  // HAS, TICKS_L/R
		"%18s %18s %18s %18s "	// HAS, VX, VY,W
		,
		"GLOBAL_ODO_X", "GLOBAL_ODO_Y", "GLOBAL_ODO_PHI_RAD", "HAS_ENCODERS",
		"LEFT_ENC_INCR_TICKS", "RIGHT_ENC_INCR_TICKS", "HAS_VELOCITIES",
		"VEL_VX_MetPerSec", "VEL_VY_MetPerSec", "ANG_SPEED_RadPerSec");
}
std::string CObservationOdometry::exportTxtDataRow() const
{
	return mrpt::format(
		"%18.5f %18.5f %18.5f "	 // GLOBAL_ODO_{x,y,phi}
		"%18i %18i %18i "  // HAS, TICKS_L/R
		"%18i %18.5f %18.5f %18.5f"	 // HAS, Vx, Vy,W
		,
		odometry.x(), odometry.y(), odometry.phi(),
		static_cast<int>(hasEncodersInfo ? 1 : 0),
		static_cast<int>(encoderLeftTicks), static_cast<int>(encoderRightTicks),
		static_cast<int>(hasVelocities ? 1 : 0), velocityLocal.vx,
		velocityLocal.vy, velocityLocal.omega);
}
