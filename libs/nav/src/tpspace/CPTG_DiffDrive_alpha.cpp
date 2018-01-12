/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header
#include <mrpt/nav/tpspace/CPTG_DiffDrive_alpha.h>
#include <mrpt/system/os.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(
	CPTG_DiffDrive_alpha, CParameterizedTrajectoryGenerator, mrpt::nav)

void CPTG_DiffDrive_alpha::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
{
	CPTG_DiffDrive_CollisionGridBased::loadFromConfigFile(cfg, sSection);

	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(
		cte_a0v_deg, double, cte_a0v, cfg, sSection);
	MRPT_LOAD_HERE_CONFIG_VAR_DEGREES_NO_DEFAULT(
		cte_a0w_deg, double, cte_a0w, cfg, sSection);
}
void CPTG_DiffDrive_alpha::saveToConfigFile(
	mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const
{
	MRPT_START
	const int WN = 25, WV = 30;
	CPTG_DiffDrive_CollisionGridBased::saveToConfigFile(cfg, sSection);

	cfg.write(
		sSection, "cte_a0v_deg", mrpt::RAD2DEG(cte_a0v), WN, WV,
		"Contant for vel profile [deg].");
	cfg.write(
		sSection, "cte_a0w_deg", mrpt::RAD2DEG(cte_a0v), WN, WV,
		"Contant for omega profile [deg].");

	MRPT_END
}

std::string CPTG_DiffDrive_alpha::getDescription() const
{
	char str[100];
	os::sprintf(
		str, 100, "CPTG_DiffDrive_alpha,av=%udeg,aw=%udeg",
		(int)RAD2DEG(cte_a0v), (int)RAD2DEG(cte_a0w));
	return std::string(str);
}

void CPTG_DiffDrive_alpha::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(in);

	switch (version)
	{
		case 0:
			in >> cte_a0v >> cte_a0w;
			break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

uint8_t CPTG_DiffDrive_alpha::serializeGetVersion() const { return 0; }
void CPTG_DiffDrive_alpha::serializeTo(mrpt::serialization::CArchive& out) const
{
	CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(out);
	out << cte_a0v << cte_a0w;
}
/*---------------------------------------------------------------
						ptgDiffDriveSteeringFunction
  ---------------------------------------------------------------*/
void CPTG_DiffDrive_alpha::ptgDiffDriveSteeringFunction(
	float alpha, float t, float x, float y, float phi, float& v, float& w) const
{
	MRPT_UNUSED_PARAM(t);
	MRPT_UNUSED_PARAM(x);
	MRPT_UNUSED_PARAM(y);
	float At_a = alpha - phi;

	while (At_a > M_PI) At_a -= (float)M_2PI;
	while (At_a < -M_PI) At_a += (float)M_2PI;

	v = V_MAX * exp(-square(At_a / cte_a0v));
	w = W_MAX * (-0.5f + (1 / (1 + exp(-At_a / cte_a0w))));
}

void CPTG_DiffDrive_alpha::loadDefaultParams()
{
	CPTG_DiffDrive_CollisionGridBased::loadDefaultParams();

	cte_a0v = mrpt::DEG2RAD(45.0);
	cte_a0w = mrpt::DEG2RAD(45.0);
}
