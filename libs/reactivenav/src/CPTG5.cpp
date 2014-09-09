/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactivenav-precomp.h" // Precomp header
#include <mrpt/reactivenav/CPTG5.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::reactivenav;
using namespace std;
using namespace mrpt::system;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPTG5::CPTG5(const TParameters<double> &params ) : CParameterizedTrajectoryGenerator (params)
{
	this->K = params["K"];
	// The constant curvature turning radius used in this PTG:
	R = V_MAX / W_MAX;
}

/*---------------------------------------------------------------
						getDescription
  ---------------------------------------------------------------*/
std::string CPTG5::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"Type#5PTG:CS");
	return std::string(str);
}

/*---------------------------------------------------------------
						PTG_Generator
  ---------------------------------------------------------------*/
void CPTG5::PTG_Generator( float alpha, float t,float x, float y, float phi, float &v, float &w )
{
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y); MRPT_UNUSED_PARAM(phi);
	const float T = 0.847f*std::sqrt(std::abs(alpha))*R/V_MAX;

	if (t< T)
	{
		// l+
		v = V_MAX;
		w = W_MAX * min( 1.0f , 1.0f - (float)exp( -square(alpha) ));
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

/*---------------------------------------------------------------
					PTG_IsIntoDomain
  ---------------------------------------------------------------*/
bool CPTG5::PTG_IsIntoDomain( float x, float y )
{
	// If signs of K and X are different, it is not into the domain:
	if ((K*x)<0)
		return false;

	if (fabs(y)>=R)
	{
		// Segmento de arriba:
		return (fabs(x)>R-0.10f);
	}
	else
	{
		// The circle at (0,R):
		return (square(x)+square(fabs(y)-(R+0.10f)))>square(R);
	}
}

