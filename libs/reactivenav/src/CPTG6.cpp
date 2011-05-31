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
CPTG6::CPTG6(const TParameters<double> &params ) : CParameterizedTrajectoryGenerator(params)
{
	cte_a0v = params["cte_a0v"];
	cte_a0w = params["cte_a0w"];
}

/*---------------------------------------------------------------
						getDescription
  ---------------------------------------------------------------*/
std::string CPTG6::getDescription() const
{
	char str[100];
	os::sprintf(str,100,"Type#6PTG,av=%udeg,aw=%udeg",(int)RAD2DEG(cte_a0v),(int)RAD2DEG(cte_a0w) );
	return std::string(str);
}

/*---------------------------------------------------------------
						PTG_Generator
  ---------------------------------------------------------------*/
void CPTG6::PTG_Generator( float alfa, float t,float x, float y, float phi, float &v, float &w )
{
    float At_a = alfa - phi;

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

/*---------------------------------------------------------------
					PTG_IsIntoDomain
  ---------------------------------------------------------------*/
bool CPTG6::PTG_IsIntoDomain( float x, float y )
{
	return true;
}

/*---------------------------------------------------------------
					lambdaFunction
  ---------------------------------------------------------------*/
void CPTG6::lambdaFunction( float x, float y, int &out_k, float &out_d )
{
	CParameterizedTrajectoryGenerator::lambdaFunction(x,y,out_k,out_d);
}
