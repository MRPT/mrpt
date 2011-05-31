/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/reactivenav.h>  // Precomp header

using namespace mrpt;
using namespace mrpt::reactivenav;
using namespace mrpt::system;

#define CTE_DIV_ALFA 2

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
	char str[100];
	os::sprintf(str,100,"Type#1PTG,circular,K=%i",(int)K );
	return std::string(str);
}


/*---------------------------------------------------------------

  ---------------------------------------------------------------*/
void CPTG1::PTG_Generator( float alfa, float t,float x, float y, float phi, float &v, float &w )
{
    // (v,w)
    v = V_MAX * sign(K);
    w = tan( alfa/CTE_DIV_ALFA ) * W_MAX * sign(K);
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
	double		R,a;

	if (y!=0)
	{
		R = (x*x+y*y)/(2*y);
		a = sign(K)*2*atan( V_MAX / (W_MAX*R) );
		k_out = alfa2index( (float)a );

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

		if (theta<0) theta += (float)M_2PI;

		d_out = (float)(theta * (fabs(R)+turningRadiusReference));
	}
	else
	{
		if (sign(x)==sign(K))
		{
			k_out = alfa2index(0);
			d_out = x;
		}
		else
		{
			k_out = alfa2index((float)M_PI);
			d_out = 1e+3;
		}
	}

	// Normalize:
	d_out = d_out / refDistance;

}
