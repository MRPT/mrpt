/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/reactivenav.h>  // Precomp header

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
	return true;
}

/*---------------------------------------------------------------
                lambdaFunction
  ---------------------------------------------------------------*/
void CPTG1::lambdaFunction( float x, float y, int &k_out, float &d_out )
{
	MRPT_TODO("Update API to return a bool for approximate results!")

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

		bool is_approx = false;
		if (std::abs(R)<Rmin)
		{
			is_approx=true;
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
		}
		else
		{
			k_out = alpha2index((float)M_PI);
			d_out = 1e+3;
		}
	}

	// Normalize:
	d_out = d_out / refDistance;

	ASSERT_ABOVEEQ_(k_out,0)
	ASSERT_BELOW_(k_out,this->m_alphaValuesCount)
}
