/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/holonomic/CHolonomicVFF.h>
#include <mrpt/utils/CStream.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

using namespace mrpt::nav;

IMPLEMENTS_SERIALIZABLE( CLogFileRecord_VFF, CHolonomicLogFileRecord,mrpt::nav )
IMPLEMENTS_SERIALIZABLE( CHolonomicVFF, CAbstractHolonomicReactiveMethod,mrpt::nav)

/*---------------------------------------------------------------
						initialize
  ---------------------------------------------------------------*/
CHolonomicVFF::CHolonomicVFF(const mrpt::utils::CConfigFileBase *INI_FILE) :
	CAbstractHolonomicReactiveMethod("VFF_CONFIG")
{
	if (INI_FILE!=NULL)
		initialize( *INI_FILE );
}

void CHolonomicVFF::initialize(const mrpt::utils::CConfigFileBase &INI_FILE)
{
	options.loadFromConfigFile(INI_FILE, getConfigFileSectionName());
}


/*---------------------------------------------------------------
						navigate
  ---------------------------------------------------------------*/
void  CHolonomicVFF::navigate(
	const mrpt::math::TPoint2D &target,
	const std::vector<double>	&obstacles,
	double			maxRobotSpeed,
	double			&desiredDirection,
	double			&desiredSpeed,
	CHolonomicLogFileRecordPtr &logRecord,
	const double    max_obstacle_dist,
	const mrpt::nav::ClearanceDiagram *clearance)
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
	if (m_enableApproachTargetSlowDown)
	{
		const double targetNearnessFactor = std::min(1.0, target.norm() / (options.TARGET_SLOW_APPROACHING_DISTANCE));
		desiredSpeed = maxRobotSpeed * std::min(obstacleNearnessFactor, targetNearnessFactor);
	}
}

void  CHolonomicVFF::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << options.TARGET_ATTRACTIVE_FORCE << options.TARGET_SLOW_APPROACHING_DISTANCE;
	}
}
void  CHolonomicVFF::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			in >> options.TARGET_ATTRACTIVE_FORCE >> options.TARGET_SLOW_APPROACHING_DISTANCE;
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}


/*---------------------------------------------------------------
					writeToStream
	Implements the writing to a CStream capability of
	  CSerializable objects
  ---------------------------------------------------------------*/
void  CLogFileRecord_VFF::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	MRPT_UNUSED_PARAM(out);
	if (version)
		*version = 0;
	else
	{
	}
}

/*---------------------------------------------------------------
					readFromStream
  ---------------------------------------------------------------*/
void  CLogFileRecord_VFF::readFromStream(mrpt::utils::CStream &in,int version)
{
	MRPT_UNUSED_PARAM(in);
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
	const int WN = 25, WV = 30;

	cfg.write(section,"TARGET_SLOW_APPROACHING_DISTANCE",TARGET_SLOW_APPROACHING_DISTANCE,   WN,WV, "For stopping gradually");
	cfg.write(section,"TARGET_ATTRACTIVE_FORCE",TARGET_ATTRACTIVE_FORCE,   WN,WV, "Dimension-less (may have to be tuned depending on the density of obstacle sampling)");

	MRPT_END
}
