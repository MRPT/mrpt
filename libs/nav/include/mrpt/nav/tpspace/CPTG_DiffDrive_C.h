/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CPTG_DiffDrive_CollisionGridBased.h>

namespace mrpt
{
  namespace nav
  {
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CPTG_DiffDrive_C, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

	/** A PTG for circular paths ("C" type PTG in papers). 
	 * - **Compatible kinematics**: differential-driven / Ackermann steering
	 * - **Compatible robot shape**: Arbitrary 2D polygon
	 * - **PTG parameters**: Use the app `ptg-configurator`
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
	 * \note [Before MRPT 1.5.0 this was named CPTG1]
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP CPTG_DiffDrive_C : public CPTG_DiffDrive_CollisionGridBased
	{
		DEFINE_SERIALIZABLE(CPTG_DiffDrive_C)
	 public:
		CPTG_DiffDrive_C() : K(0) { }
		CPTG_DiffDrive_C(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) {
			loadFromConfigFile(cfg,sSection);
		}
		virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) MRPT_OVERRIDE;
		virtual void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const MRPT_OVERRIDE;

		std::string getDescription() const MRPT_OVERRIDE;
		bool inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist = 0.10) const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void ptgDiffDriveSteeringFunction( float alpha, float t,float x, float y, float phi, float &v, float &w ) const MRPT_OVERRIDE;
		void loadDefaultParams() MRPT_OVERRIDE;

	 protected:
		/** A generation parameter */
		double K;
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CPTG_DiffDrive_C, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

  }
}

