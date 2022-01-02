/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "nav-precomp.h"  // Precomp header
//
#include <mrpt/math/wrap2pi.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/serialization/CArchive.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;

IMPLEMENTS_SERIALIZABLE(
	CPTG_DiffDrive_C, CParameterizedTrajectoryGenerator, mrpt::nav)

void CPTG_DiffDrive_C::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& cfg, const std::string& sSection)
{
	CPTG_DiffDrive_CollisionGridBased::loadFromConfigFile(cfg, sSection);

	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(K, double, cfg, sSection);
}
void CPTG_DiffDrive_C::saveToConfigFile(
	mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const
{
	MRPT_START
	const int WN = 25, WV = 30;
	CPTG_DiffDrive_CollisionGridBased::saveToConfigFile(cfg, sSection);

	cfg.write(
		sSection, "K", K, WN, WV,
		"K=+1 forward paths; K=-1 for backwards paths.");

	MRPT_END
}

void CPTG_DiffDrive_C::serializeFrom(
	mrpt::serialization::CArchive& in, uint8_t version)
{
	CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(in);

	switch (version)
	{
		case 0: in >> K; break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

uint8_t CPTG_DiffDrive_C::serializeGetVersion() const { return 0; }
void CPTG_DiffDrive_C::serializeTo(mrpt::serialization::CArchive& out) const
{
	CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(out);
	out << K;
}

std::string CPTG_DiffDrive_C::getDescription() const
{
	return mrpt::format("CPTG_DiffDrive_C,K=%i", (int)K);
}

void CPTG_DiffDrive_C::ptgDiffDriveSteeringFunction(
	[[maybe_unused]] float alpha, [[maybe_unused]] float t,
	[[maybe_unused]] float x, [[maybe_unused]] float y,
	[[maybe_unused]] float phi, [[maybe_unused]] float& v,
	[[maybe_unused]] float& w) const
{
	// (v,w)
	v = V_MAX * sign(K);
	// Use a linear mapping:  (Old was: w = tan( alpha/2 ) * W_MAX * sign(K))
	w = (alpha / M_PI) * W_MAX * sign(K);
}

bool CPTG_DiffDrive_C::PTG_IsIntoDomain(
	[[maybe_unused]] double x, [[maybe_unused]] double y) const
{
	return true;
}

bool CPTG_DiffDrive_C::inverseMap_WS2TP(
	double x, double y, int& k_out, double& d_out,
	[[maybe_unused]] double tolerance_dist) const
{
	bool is_exact = true;
	if (y != 0)
	{
		double R = (x * x + y * y) / (2 * y);
		const double Rmin = std::abs(V_MAX / W_MAX);

		double theta;

		if (K > 0)
		{
			if (y > 0) theta = atan2((double)x, fabs(R) - y);
			else
				theta = atan2((double)x, y + fabs(R));
		}
		else
		{
			if (y > 0) theta = atan2(-(double)x, fabs(R) - y);
			else
				theta = atan2(-(double)x, y + fabs(R));
		}

		// Arc length must be possitive [0,2*pi]
		mrpt::math::wrapTo2PiInPlace(theta);

		// Distance thru arc:
		d_out = (float)(theta * (fabs(R) + turningRadiusReference));

		if (std::abs(R) < Rmin)
		{
			is_exact = false;
			R = Rmin * mrpt::sign(R);
		}

		// Was: a = 2*atan( V_MAX / (W_MAX*R) );
		const double a = M_PI * V_MAX / (W_MAX * R);
		k_out = alpha2index((float)a);
	}
	else
	{
		if (sign(x) == sign(K))
		{
			k_out = alpha2index(0);
			d_out = x;
			is_exact = true;
		}
		else
		{
			k_out = m_alphaValuesCount - 1;
			d_out = 1e+3;
			is_exact = false;
		}
	}

	// Normalize:
	d_out = d_out / refDistance;

	ASSERT_GE_(k_out, 0);
	ASSERT_LT_(k_out, m_alphaValuesCount);

	return is_exact;
}

void CPTG_DiffDrive_C::loadDefaultParams()
{
	CPTG_DiffDrive_CollisionGridBased::loadDefaultParams();
	K = +1.0;
}
