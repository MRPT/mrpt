/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header

#include <mrpt/nav/holonomic/CHolonomicVFF.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace std;
using namespace mrpt::nav;

IMPLEMENTS_SERIALIZABLE(CLogFileRecord_VFF, CHolonomicLogFileRecord, mrpt::nav)
IMPLEMENTS_SERIALIZABLE(
	CHolonomicVFF, CAbstractHolonomicReactiveMethod, mrpt::nav)

/*---------------------------------------------------------------
						initialize
  ---------------------------------------------------------------*/
CHolonomicVFF::CHolonomicVFF(const mrpt::config::CConfigFileBase* INI_FILE)
	: CAbstractHolonomicReactiveMethod("CHolonomicVFF")
{
	if (INI_FILE != nullptr) initialize(*INI_FILE);
}

void CHolonomicVFF::initialize(const mrpt::config::CConfigFileBase& INI_FILE)
{
	options.loadFromConfigFile(INI_FILE, getConfigFileSectionName());
}
void CHolonomicVFF::saveConfigFile(mrpt::config::CConfigFileBase& c) const
{
	options.saveToConfigFile(c, getConfigFileSectionName());
}

/*---------------------------------------------------------------
						navigate
  ---------------------------------------------------------------*/
void CHolonomicVFF::navigate(const NavInput& ni, NavOutput& no)
{
	const auto ptg = getAssociatedPTG();
	const double ptg_ref_dist = ptg ? ptg->getRefDistance() : 1.0;

	// Create a log record for returning data.
	no.logRecord = std::make_shared<CLogFileRecord_VFF>();

	// Forces vector:
	mrpt::math::TPoint2D resultantForce(0, 0), instantaneousForce(0, 0);

	// Obstacles:
	{
		const size_t n = ni.obstacles.size();
		const double inc_ang = 2 * M_PI / n;
		double ang = -M_PI + 0.5 * inc_ang;
		for (size_t i = 0; i < n; i++, ang += inc_ang)
		{
			// Compute force strength:
			// const double mod = exp(- obstacles[i] );
			const double mod = std::min(1e6, 1.0 / ni.obstacles[i]);

			// Add repulsive force:
			instantaneousForce.x = -cos(ang) * mod;
			instantaneousForce.y = -sin(ang) * mod;
			resultantForce += instantaneousForce;
		}
	}

	const double obstcl_weight = 20.0 / ni.obstacles.size();
	resultantForce *= obstcl_weight;

	const double obstacleNearnessFactor =
		std::min(1.0, 6.0 / resultantForce.norm());

	// Target:
	ASSERT_(!ni.targets.empty());
	const auto trg = *ni.targets.rbegin();

	const double ang = atan2(trg.y, trg.x);
	const double mod = options.TARGET_ATTRACTIVE_FORCE;
	resultantForce += mrpt::math::TPoint2D(cos(ang) * mod, sin(ang) * mod);

	// Result:
	no.desiredDirection = (resultantForce.y == 0 && resultantForce.x == 0)
							  ? 0
							  : atan2(resultantForce.y, resultantForce.x);

	// Speed control: Reduction factors
	// ---------------------------------------------
	if (m_enableApproachTargetSlowDown)
	{
		const double targetNearnessFactor = std::min(
			1.0, trg.norm() /
					 (options.TARGET_SLOW_APPROACHING_DISTANCE / ptg_ref_dist));
		no.desiredSpeed =
			ni.maxRobotSpeed *
			std::min(obstacleNearnessFactor, targetNearnessFactor);
	}
}

uint8_t CHolonomicVFF::serializeGetVersion() const { return 0; }
void CHolonomicVFF::serializeTo(mrpt::serialization::CArchive& out) const
{
	out << options.TARGET_ATTRACTIVE_FORCE
		<< options.TARGET_SLOW_APPROACHING_DISTANCE;
}
void CHolonomicVFF::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	switch (version)
	{
		case 0:
		{
			in >> options.TARGET_ATTRACTIVE_FORCE >>
				options.TARGET_SLOW_APPROACHING_DISTANCE;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

uint8_t CLogFileRecord_VFF::serializeGetVersion() const { return 0; }
void CLogFileRecord_VFF::serializeTo(mrpt::serialization::CArchive& out) const
{
}

void CLogFileRecord_VFF::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	MRPT_UNUSED_PARAM(in);
	switch (version)
	{
		case 0:
		{
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

/*---------------------------------------------------------------
						TOptions
  ---------------------------------------------------------------*/
CHolonomicVFF::TOptions::TOptions()

	= default;

void CHolonomicVFF::TOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const std::string& section)
{
	MRPT_START

	// Load from config text:
	MRPT_LOAD_CONFIG_VAR(
		TARGET_SLOW_APPROACHING_DISTANCE, double, source, section);
	MRPT_LOAD_CONFIG_VAR(TARGET_ATTRACTIVE_FORCE, double, source, section);

	MRPT_END
}

void CHolonomicVFF::TOptions::saveToConfigFile(
	mrpt::config::CConfigFileBase& c, const std::string& s) const
{
	MRPT_START;

	MRPT_SAVE_CONFIG_VAR_COMMENT(
		TARGET_SLOW_APPROACHING_DISTANCE, "For stopping gradually");
	MRPT_SAVE_CONFIG_VAR_COMMENT(
		TARGET_ATTRACTIVE_FORCE,
		"Dimension-less (may have to be tuned depending on the density of "
		"obstacle sampling)");

	MRPT_END;
}
