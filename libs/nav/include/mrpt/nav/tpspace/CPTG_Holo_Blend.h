/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/utils/pimpl.h>

PIMPL_FORWARD_DECLARATION(namespace exprtk { template <typename T> class expression; })

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
		virtual ~CPTG_Holo_Blend();

		virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) MRPT_OVERRIDE;
		virtual void saveToConfigFile(mrpt::utils::CConfigFileBase &cfg,const std::string &sSection) const MRPT_OVERRIDE;
		virtual void loadDefaultParams() MRPT_OVERRIDE;
		virtual bool supportVelCmdNOP() const MRPT_OVERRIDE;
		virtual double maxTimeInVelCmdNOP(int path_k) const MRPT_OVERRIDE;

		std::string getDescription() const MRPT_OVERRIDE;
		bool inverseMap_WS2TP(double x, double y, int &out_k, double &out_d, double tolerance_dist = 0.10) const MRPT_OVERRIDE;
		bool PTG_IsIntoDomain( double x, double y ) const MRPT_OVERRIDE;
		void updateCurrentRobotVel(const mrpt::math::TTwist2D &curVelLocal) MRPT_OVERRIDE;

		/** Converts a discretized "alpha" value into a feasible motion command or action. See derived classes for the meaning of these actions */
		virtual mrpt::kinematics::CVehicleVelCmdPtr directionToMotionCommand(uint16_t k) const MRPT_OVERRIDE;
		virtual mrpt::kinematics::CVehicleVelCmdPtr getSupportedKinematicVelocityCommand() const MRPT_OVERRIDE;

		size_t getPathStepCount(uint16_t k) const MRPT_OVERRIDE;
		void getPathPose(uint16_t k, uint16_t step, mrpt::math::TPose2D &p) const MRPT_OVERRIDE;
		double getPathDist(uint16_t k, uint16_t step) const  MRPT_OVERRIDE;
		bool getPathStepForDist(uint16_t k, double dist, uint16_t &out_step) const MRPT_OVERRIDE;
		double getPathStepDuration() const MRPT_OVERRIDE;
		double getMaxLinVel() const MRPT_OVERRIDE { return V_MAX; }
		double getMaxAngVel() const MRPT_OVERRIDE { return W_MAX; }

		void updateTPObstacle(double ox, double oy, std::vector<double> &tp_obstacles) const MRPT_OVERRIDE;
		void updateTPObstacleSingle(double ox, double oy, uint16_t k, double &tp_obstacle_k) const MRPT_OVERRIDE;

		static double PATH_TIME_STEP;  //!< Duration of each PTG "step"  (default: 10e-3=10 ms)
		static double eps;             //!< Mathematical "epsilon", to detect ill-conditioned situations (e.g. 1/0) (Default: 1e-4)

	protected:
		double T_ramp_max;
		double V_MAX, W_MAX;
		double turningRadiusReference;
		mrpt::math::TTwist2D curVelLocal;

		std::string expr_V, expr_W, expr_T_ramp;

		// Compilation of user-given expressions
		PIMPL_DECLARE_TYPE(exprtk::expression<double>, m_expr_v);
		PIMPL_DECLARE_TYPE(exprtk::expression<double>, m_expr_w);
		PIMPL_DECLARE_TYPE(exprtk::expression<double>, m_expr_T_ramp);
		double m_expr_dir;  // Used as symbol "dir" in m_expr_v and m_expr_w
		void internal_init_exprtks();
		double internal_get_v(const double dir) const;  //!< Evals expr_v
		double internal_get_w(const double dir) const;  //!< Evals expr_w
		double internal_get_T_ramp(const double dir) const;  //!< Evals expr_T_ramp

		void internal_processNewRobotShape() MRPT_OVERRIDE;
		void internal_initialize(const std::string & cacheFilename = std::string(), const bool verbose = true) MRPT_OVERRIDE;
		void internal_deinitialize() MRPT_OVERRIDE;

	public:

		/** Axiliary function for computing the line-integral distance along the trajectory, handling special cases of 1/0: */
		static double calc_trans_distance_t_below_Tramp(double k2, double k4, double vxi, double vyi, double t);
		/** Axiliary function for calc_trans_distance_t_below_Tramp() and others */
		static double calc_trans_distance_t_below_Tramp_abc(double t, double a, double b, double c);

	};
	DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE(CPTG_Holo_Blend, CParameterizedTrajectoryGenerator, NAV_IMPEXP)

  }
}

