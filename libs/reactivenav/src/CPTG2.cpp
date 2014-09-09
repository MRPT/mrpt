/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactivenav-precomp.h" // Precomp header
#include <mrpt/reactivenav/CPTG2.h>
#include <mrpt/system/os.h>

using namespace mrpt;
using namespace mrpt::reactivenav;
using namespace mrpt::system;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPTG2::CPTG2(const TParameters<double> &params ) : CParameterizedTrajectoryGenerator(params)
{
	cte_a0v = params["cte_a0v"];
	cte_a0w = params["cte_a0w"];
}

/*---------------------------------------------------------------
						getDescription
  ---------------------------------------------------------------*/
std::string CPTG2::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"Type#2PTG,av=%udeg,aw=%udeg",(int)RAD2DEG(cte_a0v),(int)RAD2DEG(cte_a0w) );
	return std::string(str);
}


/*---------------------------------------------------------------
						PTG_Generator
  ---------------------------------------------------------------*/
void CPTG2::PTG_Generator( float alpha, float t,float x, float y, float phi, float &v, float &w )
{
	MRPT_UNUSED_PARAM(t); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
    float At_a = alpha - phi;

    while (At_a>M_PI) At_a -= (float) M_2PI;
    while (At_a<-M_PI) At_a += (float) M_2PI;

    v = V_MAX * exp(-square( At_a / cte_a0v ));
    w=  W_MAX * (-0.5f + (1/(1+exp(-At_a/cte_a0w))));
}

/*---------------------------------------------------------------
					PTG_IsIntoDomain
  ---------------------------------------------------------------*/
bool CPTG2::PTG_IsIntoDomain( float x, float y )
{
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	return true;
}

