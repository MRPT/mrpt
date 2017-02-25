/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CCS.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CPTG_DiffDrive_CCS,CParameterizedTrajectoryGenerator,mrpt::nav)

void CPTG_DiffDrive_CCS::loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection)
{
	CPTG_DiffDrive_CollisionGridBased ::loadFromConfigFile(cfg,sSection);
	
	MRPT_LOAD_CONFIG_VAR_NO_DEFAULT(K,double, cfg,sSection);

	// The constant curvature turning radius used in this PTG:
	R = V_MAX / W_MAX;
}
void CPTG_DiffDrive_CCS::saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const
{
	MRPT_START
	const int WN = 25, WV = 30;
	CPTG_DiffDrive_CollisionGridBased::saveToConfigFile(cfg,sSection);

	cfg.write(sSection,"K",K,   WN,WV, "K=+1 forward paths; K=-1 for backwards paths.");

	MRPT_END
}

void CPTG_DiffDrive_CCS::readFromStream(mrpt::utils::CStream &in, int version)
{
	CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(in);

	switch (version)
	{
	case 0:
		in >> K;
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPTG_DiffDrive_CCS::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version) 
	{
		*version = 0;
		return;
	}

	CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(out);
	out << K;
}

std::string CPTG_DiffDrive_CCS::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"CPTG_DiffDrive_CCS,K=%i",(int)K);
	return std::string(str);
}

void CPTG_DiffDrive_CCS::ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const
{
	MRPT_UNUSED_PARAM(phi); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	float	u = fabs(alpha) * 0.5f; //0.14758362f;  // u = atan(0.5)* alpha / PI;

	if (t< u*R/V_MAX)
	{
		// l-
		v = -V_MAX;
		w = W_MAX;
	}
	else
	if (t< (u+M_PI/2)*R/V_MAX)
	{
		// l+ pi/2
		v = V_MAX;
		w = W_MAX;
	}
	else
	{
		// s+:
		v = V_MAX;
		w = 0;
	}

	// Turn in the opposite direction??
	if (alpha<0)
		w*=-1;

	v*=K;
	w*=K;
}

bool CPTG_DiffDrive_CCS::PTG_IsIntoDomain( double x, double y ) const
{
	// If signs of K and X are different, it is into the domain:
	if ((K*x)<0)
		return true;

	if (fabs(y)>=R)
	{
		// Segmento de arriba:
		return (fabs(x)<=R);
	}
	else
	{
		// The circle at (0,R):
		return (square(x)+square(fabs(y)-R))<=R;
	}
}

void CPTG_DiffDrive_CCS::loadDefaultParams()
{
	CPTG_DiffDrive_CollisionGridBased::loadDefaultParams();
	K = +1.0;
}
