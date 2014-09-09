/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactivenav-precomp.h" // Precomp header
#include <mrpt/reactivenav/CPTG7.h>

using namespace mrpt;
using namespace mrpt::reactivenav;
using namespace mrpt::system;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPTG7::CPTG7(const TParameters<double> &params ) : CParameterizedTrajectoryGenerator(params)
{
}

/*---------------------------------------------------------------
						getDescription
  ---------------------------------------------------------------*/
std::string CPTG7::getDescription() const
{
	return std::string( format("Type#7PTG") );
}

/*---------------------------------------------------------------
						PTG_Generator
  ---------------------------------------------------------------*/
void CPTG7::PTG_Generator( float alpha, float t,float x, float y, float phi, float &v, float &w )
{
	MRPT_UNUSED_PARAM(phi); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y); MRPT_UNUSED_PARAM(t);
  	float	R  = V_MAX / W_MAX;
 	float	Ty = 2*(alpha/M_PIf)*R;

    v = V_MAX;
	w = W_MAX;
    if (alpha<0) w*=-1;

	if (fabs(x)>fabs(0.5f*Ty)) w *= -1;
    if (x>=fabs(Ty)) w = 0;
}

/*---------------------------------------------------------------
					PTG_IsIntoDomain
  ---------------------------------------------------------------*/
bool CPTG7::PTG_IsIntoDomain( float x, float y )
{
	return !( (fabs(y)>M_PI*V_MAX) || (x<M_PI*V_MAX) );
}

