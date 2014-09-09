/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "reactivenav-precomp.h" // Precomp header
#include <mrpt/reactivenav/CPTG1.h>
#include <mrpt/math/wrap2pi.h>

using namespace mrpt;
using namespace mrpt::reactivenav;
using namespace mrpt::system;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPTG1::CPTG1(const TParameters<double> &params ) :
	CParameterizedTrajectoryGenerator (params)
{
	this->K = params["K"];
}

/*---------------------------------------------------------------
						getDescription
  ---------------------------------------------------------------*/
std::string CPTG1::getDescription() const
{
	return mrpt::format("Type#1PTG,circ.arcs,K=%i",(int)K);
}


/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void CPTG1::PTG_Generator( float alpha, float t,float x, float y, float phi, float &v, float &w )
{
	MRPT_UNUSED_PARAM(t); MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y); MRPT_UNUSED_PARAM(phi);
    // (v,w)
    v = V_MAX * sign(K);
	// Use a linear mapping:  (Old was: w = tan( alpha/2 ) * W_MAX * sign(K))
    w = (alpha/M_PI) * W_MAX * sign(K); 
}

/*---------------------------------------------------------------
					PTG_IsIntoDomain
  ---------------------------------------------------------------*/
bool CPTG1::PTG_IsIntoDomain( float x, float y )
{
	MRPT_UNUSED_PARAM(x); MRPT_UNUSED_PARAM(y);
	return true;
}

bool CPTG1::inverseMap_WS2TP(float x, float y, int &k_out, float &d_out, float tolerance_dist) const
{
	MRPT_UNUSED_PARAM(tolerance_dist);
	bool is_exact = true;
	if (y!=0)
	{
		double R = (x*x+y*y)/(2*y);
		const double Rmin = std::abs(V_MAX/W_MAX);

		double theta;

		if (K>0)
		{
			if (y>0)
					theta = atan2( (double)x,fabs(R)-y );
			else	theta = atan2( (double)x,y+fabs(R) );
		}
		else
		{
			if (y>0)
					theta = atan2( -(double)x,fabs(R)-y );
			else	theta = atan2( -(double)x,y+fabs(R) );
		}

		// Arc length must be possitive [0,2*pi]
		mrpt::math::wrapTo2PiInPlace(theta);

		// Distance thru arc:
		d_out = (float)(theta * (fabs(R)+turningRadiusReference));

		if (std::abs(R)<Rmin)
		{
			is_exact=false;
			R=Rmin*mrpt::utils::sign(R);
		}
		
		//Was: a = 2*atan( V_MAX / (W_MAX*R) );
		const double a = M_PI* V_MAX / (W_MAX*R);
		k_out = alpha2index( (float)a );

	}
	else
	{
		if (sign(x)==sign(K))
		{
			k_out = alpha2index(0);
			d_out = x;
			is_exact=true;
		}
		else
		{
			k_out = alpha2index((float)M_PI);
			d_out = 1e+3;
			is_exact=false;
		}
	}

	// Normalize:
	d_out = d_out / refDistance;

	ASSERT_ABOVEEQ_(k_out,0)
	ASSERT_BELOW_(k_out,this->m_alphaValuesCount)

	return is_exact;
}
