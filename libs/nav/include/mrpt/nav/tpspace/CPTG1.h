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

	/** A PTG for circular paths ("C" type PTG in papers). 
	 * 
	 * Accepted parameters in the constructor:
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
	 public:
		/** Constructor: possible values in "params", those of CParameterizedTrajectoryGenerator plus:
			*   - K: Direction, +1 or -1
			*/
		CPTG1(const mrpt::utils::TParameters<double> &params );

		std::string getDescription() const MRPT_OVERRIDE;
		bool inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist = 0.10) const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const MRPT_OVERRIDE;

	 protected:
		/** A generation parameter */
		double K;
	};
  }
}

