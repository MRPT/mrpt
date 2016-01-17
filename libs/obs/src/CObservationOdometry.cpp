/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/system/os.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationOdometry, CObservation,mrpt::obs)

/** Constructor
 */
CObservationOdometry::CObservationOdometry( ) :
	odometry(),
	hasEncodersInfo(false),
	encoderLeftTicks(0),encoderRightTicks(0),
	hasVelocities(false),
	velocityLin(0), velocityAng(0)
{
}


/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationOdometry::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	MRPT_UNUSED_PARAM(out);
	if (version)
		*version = 1;
	else
	{
		// The data
		out << odometry
			<< sensorLabel
			<< timestamp
			// Added in V1:
			<< 	hasEncodersInfo
			<< encoderLeftTicks << encoderRightTicks
			<< hasVelocities
			<< velocityLin << velocityAng;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationOdometry::readFromStream(mrpt::utils::CStream &in, int version)
{
	MRPT_UNUSED_PARAM(in);
	switch(version)
	{
	case 0:
	case 1:
		{
			in	>> odometry
			    >> sensorLabel
			    >> timestamp;

			if (version>=1)
			{
				in 	>> hasEncodersInfo
					>> encoderLeftTicks >> encoderRightTicks
					>> hasVelocities
					>> velocityLin >> velocityAng;
			}
			else
			{
				hasEncodersInfo = false;
				encoderLeftTicks = encoderRightTicks = 0;
				hasVelocities = false;
				velocityLin = velocityAng = 0;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

void CObservationOdometry::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

	o << std::endl << "Odometry reading: " << odometry << std::endl;

	// Additional data:
	if (hasEncodersInfo)
	{
		o << format(" Encoder info: deltaL=%i deltaR=%i\n", encoderLeftTicks, encoderRightTicks );
	}
	else    o << "Encoder info: Not available!\n";

	if (hasVelocities)
	{
		o << format(" Velocity info: v=%.03f m/s  w=%.03f deg/s\n", velocityLin, RAD2DEG(velocityAng) );
	}
	else   o << "Velocity info: Not available!\n";

}

