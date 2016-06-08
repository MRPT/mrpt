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
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CPTG6, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

	/** A variation of the alpha-PTG (with fixed parameters, for now)
	  *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP  CPTG6 : public CPTG_DiffDrive_CollisionGridBased
	{
		DEFINE_SERIALIZABLE(CPTG6)
	 public:
		CPTG6() {}
		CPTG6(const mrpt::utils::TParameters<double> &params) {
			setParams(params);
		}
		void setParams(const mrpt::utils::TParameters<double> &params) MRPT_OVERRIDE;
		std::string getDescription() const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const MRPT_OVERRIDE;
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CPTG6, CParameterizedTrajectoryGenerator, NAV_IMPEXP)
  }
}

