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

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPTG3::CPTG3(const TParameters<double> &params ) :
	CParameterizedTrajectoryGenerator (params)
{
	this->K = params["K"];
	// The constant curvature turning radius used in this PTG:
	R = V_MAX / W_MAX;
}

/*---------------------------------------------------------------
						getDescription
  ---------------------------------------------------------------*/
std::string CPTG3::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"Type#3PTG:C|C,S");
	return std::string(str);
}

/*---------------------------------------------------------------
						PTG_Generator
  ---------------------------------------------------------------*/
void CPTG3::PTG_Generator( float alfa, float t,float x, float y, float phi, float &v, float &w )
{
	float	u = fabs(alfa) * 0.5f; //0.14758362f;  // u = atan(0.5)* alfa / PI;

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
	if (alfa<0)
		w*=-1;

	v*=K;
	w*=K;
}

/*---------------------------------------------------------------
					PTG_IsIntoDomain
  ---------------------------------------------------------------*/
bool CPTG3::PTG_IsIntoDomain( float x, float y )
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

/*---------------------------------------------------------------
					lambdaFunction
  ---------------------------------------------------------------*/
void CPTG3::lambdaFunction( float x, float y, int &out_k, float &out_d )
{
	CParameterizedTrajectoryGenerator::lambdaFunction(x,y,out_k,out_d);
}
