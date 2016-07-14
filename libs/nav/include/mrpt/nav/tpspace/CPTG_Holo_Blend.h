/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

namespace mrpt
{
  namespace nav
  {
	DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(CPTG_Holo_Blend, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

	/** A PTG for circular-shaped robots with holonomic kinematics.
	 * - **Compatible kinematics**: Holonomic robot capable of velocity commands with a linear interpolation ("ramp "or "blending") time. See mrpt::kinematics::CVehicleSimul_Holo
	 * - **Compatible robot shape**: Circular robots
	 * - **PTG parameters**: Use the app `ptg-configurator`
	 * 
	 * \note [New in MRPT 1.5.0]
	 *  \ingroup nav_tpspace
	 */
	class NAV_IMPEXP CPTG_Holo_Blend : public CPTG_RobotShape_Circular
	{
		DEFINE_SERIALIZABLE(CPTG_Holo_Blend)
	 public:
		CPTG_Holo_Blend();
		CPTG_Holo_Blend(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection);

		virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) MRPT_OVERRIDE;
		virtual void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const MRPT_OVERRIDE;
		virtual void loadDefaultParams() MRPT_OVERRIDE;

		std::string getDescription() const MRPT_OVERRIDE;
		bool inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist = 0.10) const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void updateCurrentRobotVel(const mrpt::math::TTwist2D &curVelLocal);

		/** Converts a discretized "alpha" value into a feasible motion command or action. See derived classes for the meaning of these actions */
		void directionToMotionCommand( uint16_t k, std::vector<double> &out_action_cmd ) const MRPT_OVERRIDE;

		size_t getPathStepCount(uint16_t k) const MRPT_OVERRIDE;
		void getPathPose(uint16_t k, uint16_t step, mrpt::math::TPose2D &p) const MRPT_OVERRIDE;
		double getPathDist(uint16_t k, uint16_t step) const  MRPT_OVERRIDE;
		bool getPathStepForDist(uint16_t k, double dist, uint16_t &out_step) const MRPT_OVERRIDE;

		void updateTPObstacle(double ox, double oy, std::vector<double> &tp_obstacles) const MRPT_OVERRIDE;

	 protected:
		double T_ramp_max;
		double V_MAX, W_MAX;
		double turningRadiusReference;
		mrpt::math::TTwist2D curVelLocal;
		double maxAllowedDirAngle; //!< [rad] (default: PI)

		void internal_processNewRobotShape() MRPT_OVERRIDE;
		void internal_initialize(const std::string & cacheFilename = std::string(), const bool verbose = true) MRPT_OVERRIDE;
		void internal_deinitialize() MRPT_OVERRIDE;
	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CPTG_Holo_Blend, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

  }
}

