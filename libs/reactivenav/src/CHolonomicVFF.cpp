/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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
						navigate
  ---------------------------------------------------------------*/
void  CHolonomicVFF::navigate(
	const mrpt::math::TPoint2D &target,
	const vector_double	&obstacles,
	double			maxRobotSpeed,
	double			&desiredDirection,
	double			&desiredSpeed,
	CHolonomicLogFileRecordPtr &logRecord)
{
	// Create a log record for returning data.
	if (!logRecord)
	{
		logRecord = CLogFileRecord_VFF::Create();
	}

	// Forces vector:
	mrpt::math::TPoint2D resultantForce(0,0),instantaneousForce(0,0);

	// Obstacles:
	{
		const size_t n = obstacles.size();
		const double inc_ang = 2*M_PI/n;
		double ang = -M_PI + 0.5*inc_ang;
		for (size_t i=0;i<n;i++, ang+=inc_ang )
		{
			// Compute force strength:
			const double mod = exp(- obstacles[i] );

			// Add repulsive force:
			instantaneousForce.x = -cos(ang) * mod;
			instantaneousForce.y = -sin(ang) * mod;
			resultantForce += instantaneousForce;
		}
	}

	// Target:
	const double ang = atan2( target.y, target.x );
	const double mod = options.TARGET_ATTRACTIVE_FORCE;
	resultantForce += mrpt::math::TPoint2D(cos(ang) * mod, sin(ang) * mod );

	// Result:
	desiredDirection = (resultantForce.y==0 && resultantForce.x==0) ? 
		0 : atan2( resultantForce.y, resultantForce.x );

	// Speed control: Reduction factors
	// ---------------------------------------------
	const double targetNearnessFactor = std::min( 1.0, target.norm()/(2*options.TARGET_SLOW_APPROACHING_DISTANCE));
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

/*---------------------------------------------------------------
						TOptions
  ---------------------------------------------------------------*/
CHolonomicVFF::TOptions::TOptions() :
	TARGET_SLOW_APPROACHING_DISTANCE ( 0.10 ),
	TARGET_ATTRACTIVE_FORCE          ( 20.0 )	
{
}

void CHolonomicVFF::TOptions::loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,const std::string &section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(TARGET_SLOW_APPROACHING_DISTANCE,double,  source,section );
	MRPT_LOAD_CONFIG_VAR(TARGET_ATTRACTIVE_FORCE,double,  source,section );

	MRPT_END
}

void CHolonomicVFF::TOptions::saveToConfigFile(const std::string &section,  mrpt::utils::CConfigFileBase &cfg ) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	cfg.write(section,"TARGET_SLOW_APPROACHING_DISTANCE",TARGET_SLOW_APPROACHING_DISTANCE,   WN,WV, "For stopping gradually");
	cfg.write(section,"TARGET_ATTRACTIVE_FORCE",TARGET_ATTRACTIVE_FORCE,   WN,WV, "Dimension-less (may have to be tuned depending on the density of obstacle sampling)");

	MRPT_END
}
