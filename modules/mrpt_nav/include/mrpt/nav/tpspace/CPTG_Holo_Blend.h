/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
#pragma once

#include <mrpt/expr/CRuntimeCompiledExpression.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

namespace mrpt::nav
{
/** A PTG for circular-shaped robots with holonomic kinematics.
 * - **Compatible kinematics**: Holonomic robot capable of velocity commands
 * with a linear interpolation ("ramp "or "blending") time. See
 * mrpt::kinematics::CVehicleSimul_Holo
 * - **Compatible robot shape**: Circular robots
 * - **PTG parameters**: Use the app `ptg-configurator`
 *
 *  \ingroup nav_tpspace
 */
class CPTG_Holo_Blend : public CPTG_RobotShape_Circular
{
  DEFINE_SERIALIZABLE(CPTG_Holo_Blend, mrpt::nav)
 public:
  CPTG_Holo_Blend();
  CPTG_Holo_Blend(const mrpt::config::CConfigFileBase& cfg, const std::string& sSection);
  ~CPTG_Holo_Blend() override;

  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& cfg, const std::string& sSection) override;
  void saveToConfigFile(
      mrpt::config::CConfigFileBase& cfg, const std::string& sSection) const override;
  void loadDefaultParams() override;
  bool supportVelCmdNOP() const override;
  double maxTimeInVelCmdNOP(int path_k) const override;

  std::string getDescription() const override;
  std::optional<std::pair<int, double>> inverseMap_WS2TP(
      double x, double y, double tolerance_dist = 0.10) const override;
  bool PTG_IsIntoDomain(double x, double y) const override;
  void onNewNavDynamicState() override;

  /** Converts a discretized "alpha" value into a feasible motion command or
   * action. See derived classes for the meaning of these actions */
  mrpt::kinematics::CVehicleVelCmd::Ptr directionToMotionCommand(uint16_t k) const override;
  mrpt::kinematics::CVehicleVelCmd::Ptr getSupportedKinematicVelocityCommand() const override;

  size_t getPathStepCount(uint16_t k) const override;
  mrpt::math::TPose2D getPathPose(uint16_t k, uint32_t step) const override;
  double getPathDist(uint16_t k, uint32_t step) const override;
  using CParameterizedTrajectoryGenerator::getPathStepForDist;
  [[nodiscard]] std::optional<uint32_t> getPathStepForDist(uint16_t k, double dist) const override;
  double getPathStepDuration() const override;
  double getMaxLinVel() const override { return V_MAX; }
  double getMaxAngVel() const override { return W_MAX; }
  void updateTPObstacle(double ox, double oy, std::vector<double>& tp_obstacles) const override;
  void updateTPObstacleSingle(
      double ox, double oy, uint16_t k, double& tp_obstacle_k) const override;

  /** Mathematical "epsilon", to detect ill-conditioned situations (e.g. 1/0) */
  static constexpr double EPSILON = 1e-4;

  /** Default value of the per-instance path time step [s] \sa setPathTimeStep */
  static constexpr double DEFAULT_PATH_TIME_STEP = 10e-3;

  /** Duration of each PTG "step" [s] (config key: `path_time_step`).
   *  Shorter steps mean a finer path discretization at a proportionally
   *  higher CPU and memory cost. \sa getPathStepDuration() */
  void setPathTimeStep(double dt);

 protected:
  double m_pathTimeStep{DEFAULT_PATH_TIME_STEP};
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
      const std::string& cacheFilename = std::string(), const bool verbose = true) override;
  void internal_deinitialize() override;

 public:
  /** Axiliary function for computing the line-integral distance along the
   * trajectory, handling special cases of 1/0: */
  static double calc_trans_distance_t_below_Tramp(
      double k2, double k4, double vxi, double vyi, double t);
  /** Axiliary function for calc_trans_distance_t_below_Tramp() and others */
  static double calc_trans_distance_t_below_Tramp_abc(double t, double a, double b, double c);
};
}  // namespace mrpt::nav
