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
/** A 3D robot shape stored as a "sliced" stack of 2D polygons, used for
 * CReactiveNavigationSystem3D
 * Depending on each PTG, only the 2D polygon or the circular radius will be
 * taken into account
 *  \ingroup nav_reactive
 */
struct TRobotShape
{
	size_t size() const { return polygons.size(); }
	void resize(size_t num_levels)
	{
		polygons.resize(num_levels);
		radius.resize(num_levels);
		heights.resize(num_levels);
	}

	const mrpt::math::CPolygon& polygon(size_t level) const
	{
		return polygons[level];
	}
	const double& getRadius(size_t level) const { return radius[level]; }
	const double& getHeight(size_t level) const { return heights[level]; }
	mrpt::math::CPolygon& polygon(size_t level) { return polygons[level]; }
	void setRadius(size_t level, double r) { radius[level] = r; }
	void setHeight(size_t level, double h) { heights[level] = h; }
	const std::vector<double>& getHeights() const { return heights; }

   private:
	std::vector<mrpt::math::CPolygon> polygons;  // The polygonal bases
	std::vector<double> radius;  // The radius of each prism.
	std::vector<double> heights;  // Heights of the prisms
};

/** See base class CAbstractPTGBasedReactive for a description and instructions
 * of use.
 * This particular implementation assumes a 3D (or "2.5D") robot shape model,
 * build as a vertical stack of "2D slices".
 *
 *  Paper describing the method:
 *  - M. Jaimez-Tarifa, J. Gonzalez-Jimenez, J.L. Blanco,
 *    "Efficient Reactive Navigation with Exact Collision Determination for 3D
 * Robot Shapes",
 *     International Journal of Advanced Robotic Systems, 2015.
 *
 * Class history:
 * - SEP/2012: First design.
 * - JUL/2013: Integrated into MRPT library.
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
 * https://github.com/MRPT/mrpt/blob/master/share/mrpt/config_files/navigation-ptgs/reactive3d_config.ini
 * \verbinclude reactive3d_config.ini
 *
 *  \sa CAbstractNavigator, CParameterizedTrajectoryGenerator,
 * CAbstractHolonomicReactiveMethod
 *  \ingroup nav_reactive
 */
class CReactiveNavigationSystem3D : public CAbstractPTGBasedReactive
{
   public:
	MRPT_MAKE_ALIGNED_OPERATOR_NEW
   public:
	/** See docs in ctor of base class */
	CReactiveNavigationSystem3D(
		CRobot2NavInterface& react_iterf_impl, bool enableConsoleOutput = true,
		bool enableLogFile = false,
		const std::string& logFileDirectory =
			std::string("./reactivenav.logs"));

	/** Destructor */
	~CReactiveNavigationSystem3D() override;

	/** Change the robot shape, which is taken into account for collision grid
	 * building. */
	void changeRobotShape(TRobotShape robotShape);

	// See base class docs:
	bool checkCollisionWithLatestObstacles(
		const mrpt::math::TPose2D& relative_robot_pose) const override;
	size_t getPTG_count() const override
	{
		ASSERT_(!m_ptgmultilevel.empty());
		return m_ptgmultilevel.size();
	}
	CParameterizedTrajectoryGenerator* getPTG(size_t i) override
	{
		ASSERT_(!m_ptgmultilevel.empty() && !m_ptgmultilevel[i].PTGs.empty());
		return m_ptgmultilevel[i].PTGs[0];  // Return for the 0'th level (ptgs
		// are replicated at each level)
	}
	const CParameterizedTrajectoryGenerator* getPTG(size_t i) const override
	{
		ASSERT_(!m_ptgmultilevel.empty() && !m_ptgmultilevel[i].PTGs.empty());
		return m_ptgmultilevel[i].PTGs[0];  // Return for the 0'th level (ptgs
		// are replicated at each level)
	}

	void loadConfigFile(const mrpt::config::CConfigFileBase& c)
		override;  // See base class docs!
	void saveConfigFile(mrpt::config::CConfigFileBase& c)
		const override;  // See base class docs!

   private:
	// ------------------------------------------------------
	//					PRIVATE DEFINITIONS
	// ------------------------------------------------------

	/** A set of PTGs of the same type, one per "height level" */
	struct TPTGmultilevel
	{
		std::vector<CParameterizedTrajectoryGenerator*> PTGs;
		mrpt::math::TPoint2D TP_Target;
		TCandidateMovementPTG holonomicmov;

		TPTGmultilevel();
		~TPTGmultilevel();
	};

	// ------------------------------------------------------
	//					PRIVATE	VARIABLES
	// ------------------------------------------------------
	/** The unsorted set of obstacles from the sensors */
	mrpt::maps::CSimplePointsMap m_WS_Obstacles_unsorted;
	/** One point cloud per 2.5D robot-shape-slice, coordinates relative to the
	 * robot local frame */
	std::vector<mrpt::maps::CSimplePointsMap> m_WS_Obstacles_inlevels;

	/** The robot 3D shape model */
	TRobotShape m_robotShape;

	/** The set of PTG-transformations to be used: indices are
	 * [ptg_index][height_index] */
	std::vector<TPTGmultilevel> m_ptgmultilevel;

	// Steps for the reactive navigation sytem.
	// ----------------------------------------------------------------------------
	void STEP1_InitPTGs() override;

	// See docs in parent class
	bool implementSenseObstacles(
		mrpt::system::TTimeStamp& obs_timestamp) override;

	// See docs in parent class
	void STEP3_WSpaceToTPSpace(
		const size_t ptg_idx, std::vector<double>& out_TPObstacles,
		mrpt::nav::ClearanceDiagram& out_clearance,
		const mrpt::math::TPose2D& rel_pose_PTG_origin_wrt_sense,
		const bool eval_clearance) override;

	/** Generates a pointcloud of obstacles, and the robot shape, to be saved in
	 * the logging record for the current timestep */
	void loggingGetWSObstaclesAndShape(CLogFileRecord& out_log) override;

};  // end class
}  // namespace mrpt::nav
