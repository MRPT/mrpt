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
	/** Trajectories with a fixed linear speed (V_MAX) and a first turning part followed by a straight segment.
	 *
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP  CPTG7 : public CPTG_DiffDrive_CollisionGridBased
	{
	 public:
		/** Constructor (this PTG has no parameters) */
		CPTG7(const mrpt::utils::TParameters<double> &params );

		std::string getDescription() const MRPT_OVERRIDE;
		bool inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist = 0.10) const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const MRPT_OVERRIDE;

	};
  }
}

