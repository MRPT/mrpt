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
#ifndef CPTG2_H
#define CPTG2_H

#include <mrpt/reactivenav/CParameterizedTrajectoryGenerator.h>

namespace mrpt
{
  namespace reactivenav
  {
	/** The alpha-PTG
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP  CPTG2 : public CParameterizedTrajectoryGenerator
	{
	 public:
			/** Constructor: possible values in "params", those of CParameterizedTrajectoryGenerator plus:
			 *   - cte_a0v, cte_a0w: Parameters of this PTG
			 */
			CPTG2(const TParameters<double> &params );

			/** The lambda function.
			  */
			void lambdaFunction( float x, float y, int &out_k, float &out_d );

			/** Gets a short textual description of the PTG and its parameters.
			*/
			std::string getDescription() const;

			bool PTG_IsIntoDomain( float x, float y );

			void PTG_Generator( float alfa, float t,float x, float y, float phi, float &v, float &w );
	 protected:
			float	cte_a0v;
			float	cte_a0w;


	};
  }
}


#endif

