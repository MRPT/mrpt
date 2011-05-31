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
CPTG7::CPTG7(const TParameters<double> &params ) : CParameterizedTrajectoryGenerator(params)
{
	cte_a0v = params["cte_a0v"];
	cte_a0w = params["cte_a0w"];
}

/*---------------------------------------------------------------
						getDescription
  ---------------------------------------------------------------*/
std::string CPTG7::getDescription() const
{
	return std::string( format("Type#7PTG,av=%udeg,aw=%udeg",(int)RAD2DEG(cte_a0v),(int)RAD2DEG(cte_a0w)) );
}

/*---------------------------------------------------------------
						PTG_Generator
  ---------------------------------------------------------------*/
void CPTG7::PTG_Generator( float alfa, float t,float x, float y, float phi, float &v, float &w )
{
  	float	R  = V_MAX / W_MAX;
 	float	Ty = 2*(alfa/M_PIf)*R;

    v = V_MAX;
	w = W_MAX;
    if (alfa<0) w*=-1;

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

/*---------------------------------------------------------------
					lambdaFunction
  ---------------------------------------------------------------*/
void CPTG7::lambdaFunction( float x, float y, int &out_k, float &out_d )
{
	CParameterizedTrajectoryGenerator::lambdaFunction(x,y,out_k,out_d);
}
