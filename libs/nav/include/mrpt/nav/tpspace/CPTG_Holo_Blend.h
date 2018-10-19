/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>
#include <mrpt/expr/CRuntimeCompiledExpression.h>

namespace mrpt::nav
{
/** A PTG for circular-shaped robots with holonomic kinematics.
 * - **Compatible kinematics**: Holonomic robot capable of velocity commands
 * with a linear interpolation ("ramp "or "blending") time. See
 * mrpt::kinematics::CVehicleSimul_Holo
 * - **Compatible robot shape**: Circular robots
 * - **PTG parameters**: Use the app `ptg-configurator`
 *
 * \note [New in MRPT 1.5.0]
 *  \ingroup nav_tpspace
 */
class CPTG_Holo_Blend : public CPTG_RobotShape_Circular
{
	DEFINE_SERIALIZABLE(CPTG_Holo_Blend)
   public:
	CPTG_Holo_Blend();
	CPTG_Holo_Blend(
		const mrpt::config::CConfigFileBase& cfg, const std::string& sSection);
	~CPTG_Holo_Blend() override;

	void loadFromConfigFile(
		const mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) override;
	void saveToConfigFile(
		mrpt::config::CConfigFileBase& cfg,
		const std::string& sSection) const override;
	void loadDefaultParams() override;
	bool supportVelCmdNOP() const override;
	double maxTimeInVelCmdNOP(int path_k) const override;

	std::string getDescription() const override;
	bool inverseMap_WS2TP(
		double x, double y, int& out_k, double& out_d,
		double tolerance_dist = 0.10) const override;
	bool PTG_IsIntoDomain(double x, double y) const override;
	void onNewNavDynamicState() override;

	/** Converts a discretized "alpha" value into a feasible motion command or
	 * action. See derived classes for the meaning of these actions */
	mrpt::kinematics::CVehicleVelCmd::Ptr directionToMotionCommand(
		uint16_t k) const override;
	mrpt::kinematics::CVehicleVelCmd::Ptr getSupportedKinematicVelocityCommand()
		const override;

	size_t getPathStepCount(uint16_t k) const override;
	void getPathPose(
		uint16_t k, uint32_t step, mrpt::math::TPose2D& p) const override;
	double getPathDist(uint16_t k, uint32_t step) const override;
	bool getPathStepForDist(
		uint16_t k, double dist, uint32_t& out_step) const override;
	double getPathStepDuration() const override;
	double getMaxLinVel() const override { return V_MAX; }
	double getMaxAngVel() const override { return W_MAX; }
	void updateTPObstacle(
		double ox, double oy, std::vector<double>& tp_obstacles) const override;
	void updateTPObstacleSingle(
		double ox, double oy, uint16_t k, double& tp_obstacle_k) const override;

	/** Duration of each PTG "step"  (default: 10e-3=10 ms) */
	static double PATH_TIME_STEP;
	/** Mathematical "epsilon", to detect ill-conditioned situations (e.g. 1/0)
	 * (Default: 1e-4) */
	static double eps;

   protected:
	double T_ramp_max{-1.0};
	double V_MAX{-1.0}, W_MAX{-1.0};
	double turningRadiusReference{0.30};

	std::string expr_V, expr_W, expr_T_ramp;
	mutable std::vector<int> m_pathStepCountCache;

	// Compilation of user-given expressions
	mrpt::expr::CRuntimeCompiledExpression m_expr_v, m_expr_w, m_expr_T_ramp;
	double m_expr_dir;  // Used as symbol "dir" in m_expr_v and m_expr_w

	/** Evals expr_v */
	double internal_get_v(const double dir) const;
	/** Evals expr_w */
	double internal_get_w(const double dir) const;
	/** Evals expr_T_ramp */
	double internal_get_T_ramp(const double dir) const;

	void internal_construct_exprs();

	void internal_processNewRobotShape() override;
	void internal_initialize(
		const std::string& cacheFilename = std::string(),
		const bool verbose = true) override;
	void internal_deinitialize() override;

   public:
	/** Axiliary function for computing the line-integral distance along the
	 * trajectory, handling special cases of 1/0: */
	static double calc_trans_distance_t_below_Tramp(
		double k2, double k4, double vxi, double vyi, double t);
	/** Axiliary function for calc_trans_distance_t_below_Tramp() and others */
	static double calc_trans_distance_t_below_Tramp_abc(
		double t, double a, double b, double c);
};
}  // namespace mrpt::nav
