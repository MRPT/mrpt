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
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CPTG1, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

	/** A PTG for circular paths ("C" type PTG in papers). 
	 * 
	 * Accepted parameters in setParams()
	 * - params["ref_distance"]: Maximum trayectory distance (meters).
	 * - params["v_max"]: Maximum linear velocity (m/s)
	 * - params["w_max"]: Maximum angular velocity (rad/s)
	 * - params["K"]: Can be "+1" for forward paths, or "-1" for backward paths.
	 * 
	 * This PT generator functions are: 
	 *
	 * \f[ v(\alpha) = V_{MAX} sign(K) \f]
	 * \f[ \omega(\alpha) = \dfrac{\alpha}{\pi} W_{MAX} sign(K) \f]
	 *
	 * So, the radius of curvature of each trajectory is constant for each "alpha" value (the trajectory parameter):
	 *
	 *  \f[ R(\alpha) = \dfrac{v}{\omega} = \dfrac{V_{MAX}}{W_{MAX}} \dfrac{\pi}{\alpha} \f]
	 *
	 * from which a minimum radius of curvature can be set by selecting the appropriate values of V_MAX and W_MAX, 
	 * knowning that \f$ \alpha \in (-\pi,\pi) \f$.
	 *
	 *  ![C-PTG path examples](PTG1_paths.png)
	 *
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP CPTG1 : public CPTG_DiffDrive_CollisionGridBased
	{
		DEFINE_SERIALIZABLE(CPTG1)
	 public:
		CPTG1() : K(0) { }
		CPTG1(const mrpt::utils::TParameters<double> &params) {
			setParams(params);
		}
		void setParams(const mrpt::utils::TParameters<double> &params) MRPT_OVERRIDE;
		std::string getDescription() const MRPT_OVERRIDE;
		bool inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist = 0.10) const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const MRPT_OVERRIDE;

	 protected:
		/** A generation parameter */
		double K;
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CPTG1, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

  }
}

