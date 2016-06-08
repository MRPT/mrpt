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
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CPTG4, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

	/** A PTG for optimal paths of type "C|C" , as named in PTG papers.
	  *  See also "Obstacle Distance for Car-Like Robots", IEEE Trans. Rob. And Autom, 1999.
	  *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP  CPTG4 : public CPTG_DiffDrive_CollisionGridBased
	{
		DEFINE_SERIALIZABLE(CPTG4)
	 public:
		CPTG4() : R(0),K(0) {}
		CPTG4(const mrpt::utils::TParameters<double> &params) {
			setParams(params);
		}
		void setParams(const mrpt::utils::TParameters<double> &params) MRPT_OVERRIDE;
		std::string getDescription() const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const MRPT_OVERRIDE;

	 protected:
		 double R,K;
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CPTG4, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

  }
}

