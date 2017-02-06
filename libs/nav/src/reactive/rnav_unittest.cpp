/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/nav/reactive/CReactiveNavigationSystem.h>
#include <mrpt/nav/reactive/CReactiveNavigationSystem3D.h>
#include <mrpt/nav/reactive/CRobot2NavInterfaceForSimulator.h>
#include <mrpt/kinematics/CVehicleSimul_DiffDriven.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>

template <typename RNAVCLASS>
void run_rnav_test(const std::string &sFilename, const std::string &sHoloMethod, const mrpt::math::TPoint2D &nav_target)
{
	using namespace std;
	using namespace mrpt;
	using namespace mrpt::nav;

	const std::string sFil = mrpt::system::find_mrpt_shared_dir() + std::string("config_files/navigation-ptgs/") + sFilename;

	if (!mrpt::system::fileExists(sFil))
	{
		cerr << "**WARNING* Skipping tests since file cannot be found: '" << sFil << "'\n";
		return;
	}

	mrpt::utils::CConfigFile cfg(sFil);
	cfg.write("CAbstractPTGBasedReactive", "holonomic_method", sHoloMethod);
	cfg.discardSavingChanges();

	struct MyDummyRobotIF : public CRobot2NavInterfaceForSimulator_DiffDriven
	{
		MyDummyRobotIF(mrpt::kinematics::CVehicleSimul_DiffDriven &sim) : CRobot2NavInterfaceForSimulator_DiffDriven(sim)
		{
		}

		bool senseObstacles(mrpt::maps::CSimplePointsMap &obstacles, mrpt::system::TTimeStamp &timestamp) override
		{
			obstacles.clear();
			timestamp = mrpt::system::now();
			return true;
		}
	};

	mrpt::kinematics::CVehicleSimul_DiffDriven robot_simul;
	MyDummyRobotIF robot2nav_if(robot_simul);

	RNAVCLASS rnav(robot2nav_if, false /*no console output*/);
	rnav.setMinLoggingLevel(mrpt::utils::LVL_ERROR); // quiet

	// Load options:
	rnav.loadConfigFile(cfg);

	// And initialize:
	rnav.initialize();

	// Nav:
	CAbstractNavigator::TNavigationParams np;
	np.target = mrpt::math::TPose2D(nav_target);
	np.targetAllowedDistance = 0.35f;

	rnav.navigate(&np);

	unsigned int MAX_ITERS = 100;
	for (unsigned int i = 0; i < MAX_ITERS; i++)
	{
		printf("[run_rnav_test] iter: %i robot_pose: %s\n",i, robot_simul.getCurrentGTPose().asString().c_str());
		// Run nav:
		rnav.navigationStep();

		EXPECT_TRUE(rnav.getCurrentState() != CAbstractNavigator::NAV_ERROR);
		if (rnav.getCurrentState() == CAbstractNavigator::IDLE)
			break;

		robot_simul.simulateOneTimeStep(0.2 /*sec*/);
	}

	EXPECT_LT((mrpt::math::TPoint2D(robot_simul.getCurrentGTPose()) - nav_target).norm(), 0.4);
	EXPECT_TRUE(rnav.getCurrentState() == CAbstractNavigator::IDLE);
}

TEST(CReactiveNavigationSystem, simple_nav_VFF) {
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>("reactive2d_config.ini", "CHolonomicVFF", mrpt::math::TPoint2D(2.0,0.4) );
}
TEST(CReactiveNavigationSystem, simple_nav_ND) {
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>("reactive2d_config.ini", "CHolonomicND", mrpt::math::TPoint2D(2.0, 0.4));
}
TEST(CReactiveNavigationSystem, simple_nav_FullEval) {
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem>("reactive2d_config.ini", "CHolonomicFullEval", mrpt::math::TPoint2D(2.0, 0.4));
}

TEST(CReactiveNavigationSystem3D, simple_nav_VFF) {
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>("reactive3d_config.ini", "CHolonomicVFF", mrpt::math::TPoint2D(2.0, 0.4));
}
TEST(CReactiveNavigationSystem3D, simple_nav_ND) {
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>("reactive3d_config.ini", "CHolonomicND", mrpt::math::TPoint2D(2.0, 0.4));
}
TEST(CReactiveNavigationSystem3D, simple_nav_FullEval) {
	run_rnav_test<mrpt::nav::CReactiveNavigationSystem3D>("reactive3d_config.ini", "CHolonomicFullEval", mrpt::math::TPoint2D(2.0, 0.4));
}
