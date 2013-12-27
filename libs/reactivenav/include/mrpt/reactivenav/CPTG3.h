/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPTG3_H
#define CPTG3_H

#include <mrpt/reactivenav/CParameterizedTrajectoryGenerator.h>

namespace mrpt
{
  namespace reactivenav
  {
	/** A PTG for optimal paths of type "C|C,S" (as named in PTG papers).
	  *  See also "Obstacle Distance for Car-Like Robots", IEEE Trans. Rob. And Autom, 1999.
	  *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP  CPTG3 : public CParameterizedTrajectoryGenerator
	{
	 public:
			/** Constructor: possible values in "params", those of CParameterizedTrajectoryGenerator plus:
			 *   - K: Direction, +1 or -1
			 */
			CPTG3(const TParameters<double> &params );

			/** The lambda function.
			  */
			void lambdaFunction( float x, float y, int &out_k, float &out_d );

			/** Gets a short textual description of the PTG and its parameters.
			*/
			std::string getDescription() const;

			bool PTG_IsIntoDomain( float x, float y );

			void PTG_Generator( float alpha, float t,float x, float y, float phi, float &v, float &w );

	 protected:
		 float		R,K;


	};
  }
}


#endif

