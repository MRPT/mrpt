/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPTG6_H
#define CPTG6_H

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

namespace mrpt
{
  namespace nav
  {
	/** A variation of the alpha-PTG (with fixed parameters, for now)
	  *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP  CPTG6 : public CParameterizedTrajectoryGenerator
	{
	 public:
			/** Constructor (this PTG has no parameters) 
			 */
			CPTG6(const mrpt::utils::TParameters<double> &params );

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

