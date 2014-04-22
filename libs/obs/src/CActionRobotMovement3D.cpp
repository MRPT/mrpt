/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers


#include <mrpt/utils/CStream.h>
#include <mrpt/slam/CActionRobotMovement3D.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;

using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CActionRobotMovement3D, CAction, mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CActionRobotMovement3D::CActionRobotMovement3D() :
	poseChange(),
	estimationMethod( emOdometry ),
	hasVelocities(6,false),
	velocities(6)
{
	velocities.assign(.0);
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CActionRobotMovement3D::~CActionRobotMovement3D()
{
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CActionRobotMovement3D::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		uint32_t		i  = static_cast<uint32_t>( estimationMethod );

		out << i;

		// The PDF:
		out << poseChange;

		out << hasVelocities << velocities;

		out << timestamp;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CActionRobotMovement3D::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t	i;

			// Read the estimation method first:
			in >> i;
			estimationMethod = static_cast<TEstimationMethod>(i);

			in >> poseChange;
			in >> hasVelocities >> velocities;

			if (version>=1)
					in >> timestamp;
			else	timestamp = INVALID_TIMESTAMP;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

