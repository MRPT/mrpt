/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPTG1_H
#define CPTG1_H

#include <mrpt/reactivenav/CParameterizedTrajectoryGenerator.h>

namespace mrpt
{
  namespace reactivenav
  {

	/** A PTG for circular paths ("C" type PTG in papers). The parameter K is related with the transformation between alpha values
			and curvature of the arcs. Let R be the radius of the circular path (the inverse of the curvature).
			Then: <br> <center><code> R = K / (v<sub>MAX</sub> tan &alpha;) </code></center>
	 *  \ingroup mrpt_reactivenav_grp
	 */
	class REACTIVENAV_IMPEXP CPTG1 : public CParameterizedTrajectoryGenerator
	{
	 public:
			/** Constructor: possible values in "params", those of CParameterizedTrajectoryGenerator plus:
			 *   - K: Direction, +1 or -1
			 */
			CPTG1(const TParameters<double> &params );

			/** The lambda function. */
			void lambdaFunction( float x, float y, int &out_k, float &out_d );

			/** Gets a short textual description of the PTG and its parameters. */
			std::string getDescription() const;


			bool PTG_IsIntoDomain( float x, float y );
			void PTG_Generator( float alpha, float t,float x, float y, float phi, float &v, float &w );

	 protected:
			/** A generation parameter */
			float	K;
	};
  }
}


#endif

