/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPTG6_H
#define CPTG6_H

#include <mrpt/reactivenav/CParameterizedTrajectoryGenerator.h>

namespace mrpt
{
  namespace reactivenav
  {
	/** A variation of the alpha-PTG (with fixed parameters, for now)
	  *  \ingroup mrpt_nav_grp
	 */
	class NAV_IMPEXP  CPTG6 : public CParameterizedTrajectoryGenerator
	{
	 public:
			/** Constructor (this PTG has no parameters) 
			 */
			CPTG6(const TParameters<double> &params );

			/** Gets a short textual description of the PTG and its parameters.
			*/
			std::string getDescription() const;

			bool PTG_IsIntoDomain( float x, float y );

			void PTG_Generator( float alpha, float t,float x, float y, float phi, float &v, float &w );
	 protected:

	};
  }
}


#endif

