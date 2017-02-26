/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservationWindSensor.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationWindSensor, CObservation,mrpt::obs)

/** Constructor
 */
CObservationWindSensor::CObservationWindSensor( ) :
	speed(0.0),
	direction(0.0)
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationWindSensor::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	MRPT_UNUSED_PARAM(out);
	if (version)
		*version = 3;
	else
	{
		// The data
		out << speed
			<< direction
			<< sensorLabel
			<< timestamp
			<< sensorPoseOnRobot;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationWindSensor::readFromStream(mrpt::utils::CStream &in, int version)
{
	//MRPT_UNUSED_PARAM(in);
	switch(version)
	{
	case 0:
	case 1:
	case 2:
	case 3:
		{
			in	>> speed
				>> direction;
			if (version>=1)
				in >> sensorLabel;
			else sensorLabel="";

			if (version>=2)
					in >> timestamp;
			else 	timestamp = INVALID_TIMESTAMP;

			if (version>=3)
					in >> sensorPoseOnRobot;
			else 	sensorPoseOnRobot = CPose3D();

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}



void CObservationWindSensor::getSensorPose( CPose3D &out_sensorPose ) const
{
	out_sensorPose = sensorPoseOnRobot;
}

void CObservationWindSensor::setSensorPose( const CPose3D &newSensorPose )
{
	sensorPoseOnRobot = newSensorPose;
}


void CObservationWindSensor::getDescriptionAsText(std::ostream &o) const
{
	CObservation::getDescriptionAsText(o);

}


