/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG3.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;
using namespace mrpt::utils;

CPTG3::CPTG3(const mrpt::utils::TParameters<double> &params ) : CPTG_DiffDrive_CollisionGridBased(params)
{
	this->K = params["K"];
	// The constant curvature turning radius used in this PTG:
	R = V_MAX / W_MAX;
}

std::string CPTG3::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"Type#3PTG:C|C,S");
	return std::string(str);
}

void CPTG3::ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const
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

bool CPTG3::PTG_IsIntoDomain( double x, double y ) const
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

