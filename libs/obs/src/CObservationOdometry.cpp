/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs.h>   // Precompiled headers

#include <mrpt/slam/CObservationOdometry.h>
#include <mrpt/system/os.h>

using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationOdometry, CObservation,mrpt::slam)

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
void  CObservationOdometry::writeToStream(CStream &out, int *version) const
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
void  CObservationOdometry::readFromStream(CStream &in, int version)
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

