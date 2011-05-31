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



#include <mrpt/slam/CActionRobotMovement3D.h>

using namespace mrpt;
using namespace mrpt::slam; using namespace mrpt::utils; using namespace mrpt::poses;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CActionRobotMovement3D, CAction, mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CActionRobotMovement3D::CActionRobotMovement3D() :
	poseChange(),
	estimationMethod( emOdometry ),
	hasVelocities(6,false),
	velocities(6,0)
{
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

