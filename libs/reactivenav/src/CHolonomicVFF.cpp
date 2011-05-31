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

#include <mrpt/reactivenav.h>  // Precomp header

//#if defined(_MSC_VER)
//	#pragma warning(disable:4267)
//#endif

#include <mrpt/poses/CPoint2D.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;
using namespace std;

using namespace mrpt::reactivenav;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord_VFF, CHolonomicLogFileRecord,mrpt::reactivenav )

/*---------------------------------------------------------------
						initialize
  ---------------------------------------------------------------*/
CHolonomicVFF::CHolonomicVFF(const mrpt::utils::CConfigFileBase *INI_FILE)
{
    if (INI_FILE!=NULL)
        initialize( *INI_FILE );
}


/*---------------------------------------------------------------
						initialize
  ---------------------------------------------------------------*/
void  CHolonomicVFF::initialize( const mrpt::utils::CConfigFileBase &INI_FILE )
{

}

/*---------------------------------------------------------------
						navigate
  ---------------------------------------------------------------*/
void  CHolonomicVFF::navigate(
				poses::CPoint2D	&target,
				vector_double	&obstacles,
				double			maxRobotSpeed,
				double			&desiredDirection,
				double			&desiredSpeed,
				CHolonomicLogFileRecordPtr &logRecord)
{
	// Create a log record for returning data.
	if (logRecord)
	{
		logRecord = CLogFileRecord_VFF::Create();
	}

        // Forces vector:
        mrpt::poses::CPoint2D  resultantForce,instantaneousForce;

        // Obstacles:
		const size_t n = obstacles.size();
        for (size_t i=0;i<n;i++)
        {
			double ang = M_PI *( -1 + 2*i/double(n) );

			// Compute force strength:
			double mod = exp(- obstacles[i] );

			// Add repulsive force:
			instantaneousForce.x( -cos(ang) * mod );
			instantaneousForce.y( -sin(ang) * mod );
			resultantForce.AddComponents( instantaneousForce );
        }

        // Target:
        double ang = atan2( target.y(), target.x() );
        double mod = 20;
        instantaneousForce.x( cos(ang) * mod );
        instantaneousForce.y( sin(ang) * mod );
        resultantForce.AddComponents( instantaneousForce );

        // Result:
        desiredDirection = atan2( resultantForce.y(), resultantForce.x() );

	// Speed control: Reduction factors
	// ---------------------------------------------
	double TARGET_SLOW_APPROACHING_DISTANCE = 0.05;
	double targetNearnessFactor = min( 1.0, target.norm()/(2*TARGET_SLOW_APPROACHING_DISTANCE));
	desiredSpeed = maxRobotSpeed * targetNearnessFactor;
}

/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CLogFileRecord_VFF::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{

	}

}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CLogFileRecord_VFF::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};


}

