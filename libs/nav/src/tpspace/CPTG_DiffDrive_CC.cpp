/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG_DiffDrive_CC.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;
using namespace mrpt::utils;

IMPLEMENTS_SERIALIZABLE(CPTG_DiffDrive_CC,CParameterizedTrajectoryGenerator,mrpt::nav)

void CPTG_DiffDrive_CC::setParams(const mrpt::utils::TParameters<double> &params)
{
	this->K = params["K"];
	CPTG_DiffDrive_CollisionGridBased::setParamsCommon(params);
	// The constant curvature turning radius used in this PTG:
	R = V_MAX / W_MAX;
}


void CPTG_DiffDrive_CC::readFromStream(mrpt::utils::CStream &in, int version)
{
	CPTG_DiffDrive_CollisionGridBased::internal_readFromStream(in);

	switch (version)
	{
	case 0:
		MRPT_TODO("continue")
		break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};
}

void CPTG_DiffDrive_CC::writeToStream(mrpt::utils::CStream &out, int *version) const
{
	if (version) 
	{
		*version = 0;
		return;
	}

	CPTG_DiffDrive_CollisionGridBased::internal_writeToStream(out);

//	out << V_MAX << W_MAX
	MRPT_TODO("continue")

}

std::string CPTG_DiffDrive_CC::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"Type#4PTG:C|C");
	return std::string(str);
}

void CPTG_DiffDrive_CC::ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const
{
	MRPT_UNUSED_PARAM(phi); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	float	u = fabs(alpha) * 0.5f; /// 6.0f;

	if (t< u*R/V_MAX)
	{
		// l-
		v = -V_MAX;
		w = W_MAX;
	}
	else
	if ( t< (u + M_PI*0.5f) * R/V_MAX )
	{
		// l+
		v = V_MAX;
		w = W_MAX;
	}
	else
	{
		// END:
		v = w = 0;
	}

	// Turn in the opposite direction??
	if (alpha<0)
		w*=-1;

	v*=K;
	w*=K;

}

bool CPTG_DiffDrive_CC::PTG_IsIntoDomain( double x, double y ) const
{
	// Aprox domain: The circle at (0,R):
	return (square(x)+square(fabs(y)-R))<=R;
}


