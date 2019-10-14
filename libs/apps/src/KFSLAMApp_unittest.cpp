/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/apps/KFSLAMApp.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

TEST(KFSLAMApp, EKF_SLAM_6D)
{
	using namespace std::string_literals;
	const auto ini_fil =
		mrpt::UNITTEST_BASEDIR +
		"/share/mrpt/config_files/kf-slam/EKF-SLAM_6D_test.ini"s;
	EXPECT_TRUE(mrpt::system::fileExists(ini_fil));

	const auto rawlog_fil =
		mrpt::UNITTEST_BASEDIR + "/share/mrpt/datasets/kf-slam_6D_demo.rawlog"s;
	EXPECT_TRUE(mrpt::system::fileExists(rawlog_fil));

	try
	{
		mrpt::apps::KFSLAMApp app;
		app.setMinLoggingLevel(mrpt::system::LVL_ERROR);

		const char* argv[] = {"kf-slam", ini_fil.c_str(), rawlog_fil.c_str()};
		const int argc = sizeof(argv) / sizeof(argv[0]);

		app.initialize(argc, argv);

		app.params.write("MappingApplication", "SHOW_3D_LIVE", false);
		app.params.write("MappingApplication", "SAVE_3D_SCENES", true);

		// Output 3D scenes to a temporary directory:
		app.params.write(
			"MappingApplication", "logOutput_dir",
			mrpt::system::getTempFileName() + "_dir"s);

		app.run();

		// Check results:
		EXPECT_LT(app.loc_error_wrt_gt, 0.1);
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		GTEST_FAIL();
	}
}
