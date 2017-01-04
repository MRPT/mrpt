/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers


#include <mrpt/utils/CStream.h>
#include <mrpt/obs/CObservationRobotPose.h>
#include <mrpt/system/os.h>
#include <mrpt/math/matrix_serialization.h>

using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;


// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRobotPose, CObservation,mrpt::obs)

/** Default constructor */
CObservationRobotPose::CObservationRobotPose( )
{
}


/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRobotPose::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << pose;
		out << sensorLabel
			<< timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CObservationRobotPose::readFromStream(mrpt::utils::CStream &in, int version)
{
	switch(version)
	{
	case 0:
		{
			in >> pose;
			in >> sensorLabel
			   >> timestamp;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

}

void CObservationRobotPose::getSensorPose( CPose3D &out_sensorPose ) const
{ 
	out_sensorPose = sensorPose; 
}

void CObservationRobotPose::setSensorPose( const CPose3D &newSensorPose )
{ 
	sensorPose = newSensorPose; 
}

void CObservationRobotPose::getDescriptionAsText(std::ostream &o) const
{
	using namespace std;
	CObservation::getDescriptionAsText(o);

	o << "Sensor pose: " << sensorPose << endl;
	o << "Pose: " << pose.asString() << endl;
}
