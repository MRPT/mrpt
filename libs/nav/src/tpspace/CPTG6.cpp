+---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header
#include <mrpt/nav/tpspace/CPTG6.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::system;
using namespace mrpt::utils;

CPTG6::CPTG6(const mrpt::utils::TParameters<double> &params ) : CPTG_DiffDrive_CollisionGridBased(params)
{
}

std::string CPTG6::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"Type#6PTG" );
	return std::string(str);
}

void CPTG6::ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const
{
	MRPT_UNUSED_PARAM(t); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	float At_a = alpha - phi;

	while (At_a>M_PI) At_a -= (float) M_2PI;
	while (At_a<-M_PI) At_a += (float) M_2PI;

	v = V_MAX * (1.0f - 0.95f * square( At_a / (float)M_PI ) );
	if (At_a>0.01f)
		w = W_MAX;
	else
		if (At_a<0.01f)
				w = -W_MAX;
		else w = 0;
}

bool CPTG6::PTG_IsIntoDomain( double x, double y ) const
{
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	return true;
}
