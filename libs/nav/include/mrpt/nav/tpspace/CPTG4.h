/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>

namespace mrpt
{
  namespace nav
  {
	/** A PTG for optimal paths of type "C|C" , as named in PTG papers.
	  *  See also "Obstacle Distance for Car-Like Robots", IEEE Trans. Rob. And Autom, 1999.
	  *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP  CPTG4 : public CPTG_DiffDrive_CollisionGridBased
	{
	 public:
		/** Constructor: possible values in "params", those of CParameterizedTrajectoryGenerator plus:
			*   - K: Direction, +1 or -1
			*/
		CPTG4(const mrpt::utils::TParameters<double> &params );

		std::string getDescription() const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const MRPT_OVERRIDE;

	 protected:
		 double R,K;
	};
  }
}

