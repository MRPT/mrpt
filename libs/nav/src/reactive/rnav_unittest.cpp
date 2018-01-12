/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>
#include <mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>

using mrpt::math::TPoint2D;

template <typename RNAVCLASS>
void run_rnav_test(
	const std::string& sFilename, const std::string& sHoloMethod,
	const TPoint2D& nav_target, const TPoint2D& world_topleft,
	const TPoint2D& world_rightbottom,
	const TPoint2D& block_obstacle_topleft = TPoint2D(0, 0),
	const TPoint2D& block_obstacle_rightbottom = TPoint2D(0, 0))
{
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::nav;

	const std::string sFil = mrpt::system::find_mrpt_shared_dir() +
							 std::string("config_files/navigation-ptgs/") +
							 sFilename;

	if (!mrpt::system::fileExists(sFil))
	{
		cerr << "**WARNING* Skipping tests since file cannot be found: '"
			 << sFil << "'\n";
		return;
	}

	mrpt::config::CConfigFile cfg(sFil);
	cfg.write("CAbstractPTGBasedReactive", "holonomic_method", sHoloMethod);
	cfg.discardSavingChanges();

	// Create a grid map with a synthetic test environment with a simple
	// obstacle:
	mrpt::maps::COccupancyGridMap2D grid;
	grid.setSize(
		world_topleft.x, world_rightbottom.x, world_rightbottom.y,
		world_topleft.y, 0.10f /*resolution*/);
	grid.fill(0.9f);

	// Create obstacle:
	{
		const int xi0 = grid.x2idx(block_obstacle_topleft.x),
				  xi1 = grid.x2idx(block_obstacle_rightbottom.x);
		const int yi0 = grid.y2idx(block_obstacle_rightbottom.y),
				  yi1 = grid.y2idx(block_obstacle_topleft.y);

		for (int xi = xi0; xi < xi1; xi++)
		{
			for (int yi = yi0; yi < yi1; yi++)
			{
				grid.setCell(xi, yi, 0);
			}
		}
	}

	struct MyDummyRobotIF : public CRobot2NavInterfaceForSimulator_DiffDriven
	{
		mrpt::maps::COccupancyGridMap2D& m_grid;

		MyDummyRobotIF(
			mrpt::kinematics::CVehicleSimul_DiffDriven& sim,
			mrpt::maps::COccupancyGridMap2D& grid)
			: CRobot2NavInterfaceForSimulator_DiffDriven(sim), m_grid(grid)
		{
			this->setMinLoggingLevel(
				mrpt::system::LVL_ERROR);  // less verbose output for tests
		}

		void sendNavigationStartEvent() override {}
		void sendNavigationEndEvent() override {}
		bool senseObstacles(
			mrpt::maps::CSimplePointsMap& obstacles,
			mrpt::system::TTimeStamp& timestamp) override
		{
			obstacles.clear();
			timestamp = mrpt::system::now();

			mrpt::math::TPose2D curPose, odomPose;
			std::string pose_frame_id;
			mrpt::math::TTwist2D curVel;
			mrpt::system::TTimeStamp pose_tim;
			getCurrentPoseAndSpeeds(
				curPose, curVel, pose_tim, odomPose, pose_frame_id);

			mrpt::obs::CObservation2DRangeScan scan;
			scan.aperture = mrpt::DEG2RAD(270.0);
			scan.maxRange = 20.0;
			scan.sensorPose.z(0.4);  // height of the lidar (important! it must
			// intersect with the robot height)

			m_grid.laserScanSimulator(
				scan, mrpt::poses::CPose2D(curPose), 0.4f, 180);

			obstacles.insertionOptions.minDistBetweenLaserPoints = .0;
			obstacles.loadFromRangeScan(scan);

			return true;
		}
	};

	mrpt::kinematics::CVehicleSimul_DiffDriven robot_simul;
	MyDummyRobotIF robot2nav_if(robot_simul, grid);

	RNAVCLASS rnav(robot2nav_if, false /*no console output*/);
	// Logging:
	{
		rnav.enableTimeLog(false);
#ifdef _DEBUG
		rnav.setMinLoggingLevel(mrpt::system::LVL_DEBUG);
#else
		rnav.setMinLoggingLevel(mrpt::system::LVL_ERROR);  // quiet
#endif
		const std::string sTmpFil = mrpt::system::getTempFileName();
		const std::string sTmpDir = mrpt::system::extractFileDirectory(sTmpFil);
		// printf("[run_rnav_test] navlog dir: `%s`\n", sTmpDir.c_str());
		rnav.setLogFileDirectory(sTmpDir);
		rnav.enableLogFile(true);
	}

	// Load options:
	rnav.loadConfigFile(cfg);

	// And initialize:
	rnav.initialize();

	// Nav:
	CAbstractNavigator::TNavigationParams np;
	np.target.target_coords = mrpt::math::TPose2D(nav_target);
	np.target.targetAllowedDistance = 0.35f;

	rnav.navigate(&np);

	unsigned int MAX_ITERS = 200;
	for (unsigned int i = 0; i < MAX_ITERS; i++)
	{
		// printf("[run_rnav_test] iter: %i robot_pose: %s\n",i,
		// robot_simul.getCurrentGTPose().asString().c_str());
		// Run nav:
		rnav.navigationStep();

		EXPECT_TRUE(rnav.getCurrentState() != CAbstractNavigator::NAV_ERROR);
		if (rnav.getCurrentState() == CAbstractNavigator::IDLE) break;

		robot_simul.simulateOneTimeStep(0.2 /*sec*/);
	}

	EXPECT_LT(
		(TPoint2D(robot_simul.getCurrentGTPose()) - nav_target).norm(), 0.4);
	EXPECT_TRUE(rnav.getCurrentState() == CAbstractNavigator::IDLE);

	const_cast<mrpt::system::CTimeLogger&>(rnav.getTimeLogger())
		.clear(true);  // do not show timelog table to console
	const_cast<mrpt::system::CTimeLogger&>(rnav.getDelaysTimeLogger())
		.clear(true);
}

const TPoint2D no_obs_trg(2.0, 0.4), no_obs_topleft(-10, 10),
	no_obs_bottomright(10, -10);
const TPoint2D with_obs_trg(9.0, 4.0), with_obs_topleft(-10, 10),
	with_obs_bottomright(30, -10), obs_tl(4.0, 2.0), obs_br(5.0, -2.0);

TEST(CReactiveNavigationSystem, no_obstacle_nav_VFF)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>(
		"reactive2d_config.ini", "CHolonomicVFF", no_obs_trg, no_obs_topleft,
		no_obs_bottomright);
}
TEST(CReactiveNavigationSystem, no_obstacle_nav_ND)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>(
		"reactive2d_config.ini", "CHolonomicND", no_obs_trg, no_obs_topleft,
		no_obs_bottomright);
}
TEST(CReactiveNavigationSystem, no_obstacle_nav_FullEval)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>(
		"reactive2d_config.ini", "CHolonomicFullEval", no_obs_trg,
		no_obs_topleft, no_obs_bottomright);
}

TEST(CReactiveNavigationSystem3D, no_obstacle_nav_VFF)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>(
		"reactive3d_config.ini", "CHolonomicVFF", no_obs_trg, no_obs_topleft,
		no_obs_bottomright);
}
TEST(CReactiveNavigationSystem3D, no_obstacle_nav_ND)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>(
		"reactive3d_config.ini", "CHolonomicND", no_obs_trg, no_obs_topleft,
		no_obs_bottomright);
}
TEST(CReactiveNavigationSystem3D, no_obstacle_nav_FullEval)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>(
		"reactive3d_config.ini", "CHolonomicFullEval", no_obs_trg,
		no_obs_topleft, no_obs_bottomright);
}

TEST(CReactiveNavigationSystem, with_obstacle_nav_VFF)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>(
		"reactive2d_config.ini", "CHolonomicVFF", with_obs_trg,
		with_obs_topleft, with_obs_bottomright, obs_tl, obs_br);
}
TEST(CReactiveNavigationSystem, with_obstacle_nav_ND)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>(
		"reactive2d_config.ini", "CHolonomicND", with_obs_trg, with_obs_topleft,
		with_obs_bottomright, obs_tl, obs_br);
}
TEST(CReactiveNavigationSystem, with_obstacle_nav_FullEval)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>(
		"reactive2d_config.ini", "CHolonomicFullEval", with_obs_trg,
		with_obs_topleft, with_obs_bottomright, obs_tl, obs_br);
}

TEST(CReactiveNavigationSystem3D, with_obstacle_nav_VFF)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>(
		"reactive3d_config.ini", "CHolonomicVFF", with_obs_trg,
		with_obs_topleft, with_obs_bottomright, obs_tl, obs_br);
}
TEST(CReactiveNavigationSystem3D, with_obstacle_nav_ND)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>(
		"reactive3d_config.ini", "CHolonomicND", with_obs_trg, with_obs_topleft,
		with_obs_bottomright, obs_tl, obs_br);
}
TEST(CReactiveNavigationSystem3D, with_obstacle_nav_FullEval)
{
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>(
		"reactive3d_config.ini", "CHolonomicFullEval", with_obs_trg,
		with_obs_topleft, with_obs_bottomright, obs_tl, obs_br);
}
