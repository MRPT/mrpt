/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef CPTG2_H
#define CPTG2_H

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

namespace mrpt
{
  namespace nav
  {
	/** The "alpha-PTG", as named in PTG papers.
	 * 
	 * Accepted parameters in the constructor:
	 * - params["ref_distance"]: Maximum trayectory distance (meters).
	 * - params["v_max"]: Maximum linear velocity (m/s)
	 * - params["w_max"]: Maximum angular velocity (rad/s)
	 * - params["cte_a0v"]: Constant related to the generation of linear velocities, read below (in radians).
	 * - params["cte_a0w"]: Constant related to the generation of angular velocities, read below  (in radians).
	 * 
	 * This PT generator functions are: 
	 *
	 * \f[ v(\alpha) = V_{MAX} e^{ -\left( \dfrac{\alpha-\phi}{cte_{a0v}} \right)^2} \f]
	 * \f[ \omega(\alpha) = W_{MAX} \left( -\dfrac{1}{2} +\dfrac{1}{1+ e^{ - \dfrac{\alpha-\phi}{cte_{a0w}} } } \right) \f]
	 *
	 * So, the radius of curvature of each trajectory is NOT constant for each "alpha" value in this PTG:
	 *
	 *  ![C-PTG path examples](PTG2_paths.png)
	 *
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP  CPTG2 : public CParameterizedTrajectoryGenerator
	{
	 public:
			/** Constructor: possible values in "params", those of CParameterizedTrajectoryGenerator plus:
			 *   - cte_a0v, cte_a0w: Parameters of this PTG (both are angles in radians).
			 */
			CPTG2(const mrpt::utils::TParameters<double> &params );

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

