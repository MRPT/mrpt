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

#include "CAbstractPTGBasedReactive.h"

namespace mrpt::nav
{
/** \defgroup nav_reactive Reactive navigation classes
 * \ingroup mrpt_nav_grp
 */

/** \brief 2-D PTG-based reactive navigation system for robots with a polygonal
 * or circular footprint.
 *
 * Implements CAbstractPTGBasedReactive for flat-terrain mobile robots. The
 * robot shape is defined either as a polygon (changeRobotShape()) or as a
 * circle radius (changeRobotCircularShapeRadius()). Obstacles are gathered
 * from the CRobot2NavInterface callbacks and represented as a 2-D point cloud
 * in the local robot frame before being projected into TP-Space.
 *
 * Configuration is loaded from an INI-like file; a template can be generated
 * with saveConfigFile() on a default-constructed object.
 *
 * See base class CAbstractPTGBasedReactive for a description and instructions
 * of use.
 * This particular implementation assumes a 2D robot shape which can be
 * polygonal or circular (depending on the selected PTGs).
 *
 * Publications:
 *  - Blanco, Jose-Luis, Javier Gonzalez, and Juan-Antonio Fernandez-Madrigal.
 * ["Extending obstacle avoidance methods through multiple parameter-space
 * transformations"](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.190.4672&rep=rep1&type=pdf).
 * Autonomous Robots 24.1 (2008): 29-48.
 *
 * Class history:
 * - 17/JUN/2004: First design.
 * - 16/SEP/2004: Totally redesigned, according to document "MultiParametric
 * Based Space Transformation for Reactive Navigation"
 * - 29/SEP/2005: Totally rewritten again, for integration into MRPT library and
 * according to the ICRA paper.
 * - 17/OCT/2007: Whole code updated to accommodate to MRPT 0.5 and make it
 * portable to Linux.
 * - DEC/2013: Code refactoring between this class and
 * CAbstractHolonomicReactiveMethod
 * - FEB/2017: Refactoring of all parameters for a consistent organization in
 * sections by class names (MRPT 1.5.0)
 *
 * This class requires a number of parameters which are usually provided via an
 * external config (".ini") file.
 * Alternatively, a memory-only object can be used to avoid physical files, see
 * mrpt::config::CConfigFileMemory.
 *
 * A template config file can be generated at any moment by the user by calling
 * saveConfigFile() with a default-constructed object.
 *
 * Next we provide a self-documented template config file; or see it online:
 * https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/navigation-ptgs/reactive2d_config.ini
 * \verbinclude reactive2d_config.ini
 *
 *  \sa CAbstractNavigator, CParameterizedTrajectoryGenerator,
 * CAbstractHolonomicReactiveMethod
 *  \ingroup nav_reactive
 */
class CReactiveNavigationSystem : public CAbstractPTGBasedReactive
{
 public:
  /** \brief Constructor.
   *
   * \param[in] react_iterf_impl   User-supplied robot interface (callbacks).
   * \param[in] enableConsoleOutput If true, log messages are echoed to stdout.
   * \param[in] enableLogFile      If true, detailed per-step log files are
   *            written to \a logFileDirectory.
   * \param[in] logFileDirectory   Directory for per-step log files.
   */
  CReactiveNavigationSystem(
      CRobot2NavInterface& react_iterf_impl,
      bool enableConsoleOutput = true,
      bool enableLogFile = false,
      const std::string& logFileDirectory = std::string("./reactivenav.logs"));

  /** \brief Destructor. */
  ~CReactiveNavigationSystem() override;

  /** \brief Sets the robot footprint as a 2-D polygon, used by PTGs for
   * collision checking.
   * \param[in] shape The robot convex hull polygon (local robot frame).
   */
  void changeRobotShape(const mrpt::math::CPolygon& shape);

  /** \brief Sets the robot footprint as a circle, used by PTGs for collision
   * checking.
   * \param[in] R Circle radius in meters.
   */
  void changeRobotCircularShapeRadius(const double R);

  // See base class docs:
  size_t getPTG_count() const override { return PTGs.size(); }
  CParameterizedTrajectoryGenerator* getPTG(size_t i) override
  {
    ASSERT_(i < PTGs.size());
    return PTGs[i].get();
  }
  const CParameterizedTrajectoryGenerator* getPTG(size_t i) const override
  {
    ASSERT_(i < PTGs.size());
    return PTGs[i].get();
  }
  bool checkCollisionWithLatestObstacles(
      const mrpt::math::TPose2D& relative_robot_pose) const override;

  struct TReactiveNavigatorParams : public mrpt::config::CLoadableOptions
  {
    double min_obstacles_height{0.0},
        max_obstacles_height{10.0};  // The range of "z" coordinates for obstacles
    // to be considered

    void loadFromConfigFile(const mrpt::config::CConfigFileBase& c, const std::string& s) override;
    void saveToConfigFile(mrpt::config::CConfigFileBase& c, const std::string& s) const override;
    TReactiveNavigatorParams();
  };

  TReactiveNavigatorParams params_reactive_nav;

  void loadConfigFile(const mrpt::config::CConfigFileBase& c) override;  // See base class docs!
  void saveConfigFile(mrpt::config::CConfigFileBase& c) const override;  // See base class docs!

 private:
  /** The list of PTGs to use for navigation */
  std::vector<CParameterizedTrajectoryGenerator::Ptr> PTGs;

  // Steps for the reactive navigation sytem.
  // ----------------------------------------------------------------------------
  void initPTGs() override;

  // See docs in parent class
  bool implementSenseObstacles(mrpt::system::TTimeStamp& obs_timestamp) override;

 protected:
  /** The robot 2D shape model. Only one of `robot_shape` or
   * `robot_shape_circular_radius` will be used in each PTG */
  mrpt::math::CPolygon m_robotShape;
  /** Radius of the robot if approximated as a circle. Only one of
   * `robot_shape` or `robot_shape_circular_radius` will be used in each PTG
   */
  double m_robotShapeCircularRadius;

  /** Generates a pointcloud of obstacles, and the robot shape, to be saved in
   * the logging record for the current timestep */
  void loggingGetWSObstaclesAndShape(CLogFileRecord& out_log) override;

  /** The obstacle points, as seen from the local robot frame. */
  mrpt::maps::CSimplePointsMap m_WS_Obstacles;
  /** Obstacle points, before filtering (if filtering is enabled). */
  mrpt::maps::CSimplePointsMap m_WS_Obstacles_original;
  // See docs in parent class
  void transformToTPSpace(
      size_t ptg_idx,
      std::vector<double>& out_TPObstacles,
      mrpt::nav::ClearanceDiagram& out_clearance,
      const mrpt::math::TPose2D& rel_pose_PTG_origin_wrt_sense,
      const bool eval_clearance) override;

};  // end class
}  // namespace mrpt::nav
