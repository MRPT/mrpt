/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include "CAbstractPTGBasedReactive.h"

namespace mrpt::nav
{
/** \defgroup nav_reactive Reactive navigation classes
 * \ingroup mrpt_nav_grp
 */

/** See base class CAbstractPTGBasedReactive for a description and instructions
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
 * - 17/OCT/2007: Whole code updated to accomodate to MRPT 0.5 and make it
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
	MRPT_MAKE_ALIGNED_OPERATOR_NEW
   public:
	/** See docs in ctor of base class */
	CReactiveNavigationSystem(
		CRobot2NavInterface& react_iterf_impl, bool enableConsoleOutput = true,
		bool enableLogFile = false,
		const std::string& logFileDirectory =
			std::string("./reactivenav.logs"));

	/** Destructor
	 */
	~CReactiveNavigationSystem() override;

	/** Defines the 2D polygonal robot shape, used for some PTGs for collision
	 * checking. */
	void changeRobotShape(const mrpt::math::CPolygon& shape);
	/** Defines the 2D circular robot shape radius, used for some PTGs for
	 * collision checking. */
	void changeRobotCircularShapeRadius(const double R);

	// See base class docs:
	size_t getPTG_count() const override { return PTGs.size(); }
	CParameterizedTrajectoryGenerator* getPTG(size_t i) override
	{
		ASSERT_(i < PTGs.size());
		return PTGs[i];
	}
	const CParameterizedTrajectoryGenerator* getPTG(size_t i) const override
	{
		ASSERT_(i < PTGs.size());
		return PTGs[i];
	}
	bool checkCollisionWithLatestObstacles(
		const mrpt::math::TPose2D& relative_robot_pose) const override;

	struct TReactiveNavigatorParams : public mrpt::config::CLoadableOptions
	{
		double min_obstacles_height{0.0},
			max_obstacles_height{
				10.0};  // The range of "z" coordinates for obstacles
		// to be considered

		void loadFromConfigFile(
			const mrpt::config::CConfigFileBase& c,
			const std::string& s) override;
		void saveToConfigFile(
			mrpt::config::CConfigFileBase& c,
			const std::string& s) const override;
		TReactiveNavigatorParams();
	};

	TReactiveNavigatorParams params_reactive_nav;

	void loadConfigFile(const mrpt::config::CConfigFileBase& c)
		override;  // See base class docs!
	void saveConfigFile(mrpt::config::CConfigFileBase& c)
		const override;  // See base class docs!

   private:
	/** The list of PTGs to use for navigation */
	std::vector<CParameterizedTrajectoryGenerator*> PTGs;

	// Steps for the reactive navigation sytem.
	// ----------------------------------------------------------------------------
	void STEP1_InitPTGs() override;

	// See docs in parent class
	bool implementSenseObstacles(
		mrpt::system::TTimeStamp& obs_timestamp) override;

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
	void STEP3_WSpaceToTPSpace(
		const size_t ptg_idx, std::vector<double>& out_TPObstacles,
		mrpt::nav::ClearanceDiagram& out_clearance,
		const mrpt::math::TPose2D& rel_pose_PTG_origin_wrt_sense,
		const bool eval_clearance) override;

};  // end class
}  // namespace mrpt::nav
