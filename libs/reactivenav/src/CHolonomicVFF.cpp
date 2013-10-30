/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
			//const double mod = exp(- obstacles[i] );
			const double mod = std::min(1e6, 1.0/ obstacles[i] );

			// Add repulsive force:
			instantaneousForce.x = -cos(ang) * mod;
			instantaneousForce.y = -sin(ang) * mod;
			resultantForce += instantaneousForce;
		}
	}

	const double obstcl_weight = 20.0/obstacles.size();
	resultantForce *= obstcl_weight;

	const double obstacleNearnessFactor = std::min( 1.0, 6.0/resultantForce.norm());

	// Target:
	const double ang = atan2( target.y, target.x );
	const double mod = options.TARGET_ATTRACTIVE_FORCE;
	resultantForce += mrpt::math::TPoint2D(cos(ang) * mod, sin(ang) * mod );

	// Result:
	desiredDirection = (resultantForce.y==0 && resultantForce.x==0) ?
		0 : atan2( resultantForce.y, resultantForce.x );

	// Speed control: Reduction factors
	// ---------------------------------------------
	const double targetNearnessFactor = std::min( 1.0, target.norm()/(options.TARGET_SLOW_APPROACHING_DISTANCE));
	//desiredSpeed = maxRobotSpeed * std::min(obstacleNearnessFactor, targetNearnessFactor);
	desiredSpeed = std::min(obstacleNearnessFactor, targetNearnessFactor);
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

void CHolonomicVFF::TOptions::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg , const std::string &section) const
{
	MRPT_START
	const int WN = 40, WV = 20;

	cfg.write(section,"TARGET_SLOW_APPROACHING_DISTANCE",TARGET_SLOW_APPROACHING_DISTANCE,   WN,WV, "For stopping gradually");
	cfg.write(section,"TARGET_ATTRACTIVE_FORCE",TARGET_ATTRACTIVE_FORCE,   WN,WV, "Dimension-less (may have to be tuned depending on the density of obstacle sampling)");

	MRPT_END
}
