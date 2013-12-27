/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPTG2_H
#define CPTG2_H

#include <mrpt/reactivenav/CParameterizedTrajectoryGenerator.h>

namespace mrpt
{
  namespace reactivenav
  {
	/** The "alpha-PTG", as named in PTG papers.
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP  CPTG2 : public CParameterizedTrajectoryGenerator
	{
	 public:
			/** Constructor: possible values in "params", those of CParameterizedTrajectoryGenerator plus:
			 *   - cte_a0v, cte_a0w: Parameters of this PTG (both are angles in radians).
			 */
			CPTG2(const TParameters<double> &params );

			/** The lambda function.
			  */
			void lambdaFunction( float x, float y, int &out_k, float &out_d );

			/** Gets a short textual description of the PTG and its parameters.
			*/
			std::string getDescription() const;

			bool PTG_IsIntoDomain( float x, float y );

			void PTG_Generator( float alpha, float t,float x, float y, float phi, float &v, float &w );
	 protected:
			float	cte_a0v;
			float	cte_a0w;


	};
  }
}


#endif

