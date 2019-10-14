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
#include <functional>

using config_changer_t = std::function<void(mrpt::config::CConfigFileBase&)>;

void generic_kf_slam_test(
	const std::string& ini_filename, const std::string& rawlog_filename,
	config_changer_t cfg_changer)
{
	using namespace std::string_literals;

	const auto ini_fil = mrpt::UNITTEST_BASEDIR +
						 "/share/mrpt/config_files/kf-slam/"s + ini_filename;
	EXPECT_TRUE(mrpt::system::fileExists(ini_fil));

	const auto rawlog_fil =
		mrpt::UNITTEST_BASEDIR + "/share/mrpt/datasets/"s + rawlog_filename;
	EXPECT_TRUE(mrpt::system::fileExists(rawlog_fil));

	try
	{
		mrpt::apps::KFSLAMApp app;
		app.setMinLoggingLevel(mrpt::system::LVL_ERROR);

		const char* argv[] = {"kf-slam", ini_fil.c_str(), rawlog_fil.c_str()};
		const int argc = sizeof(argv) / sizeof(argv[0]);

		app.initialize(argc, argv);
		cfg_changer(app.params);
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

TEST(KFSLAMApp, EKF_SLAM_3D)
{
	generic_kf_slam_test(
		"EKF-SLAM_6D_test.ini", "kf-slam_6D_demo.rawlog",
		[](mrpt::config::CConfigFileBase& c) {
			using namespace std::string_literals;
			c.write("MappingApplication", "SHOW_3D_LIVE", false);
			c.write("MappingApplication", "SAVE_3D_SCENES", true);
			// Output 3D scenes to a temporary directory:
			c.write(
				"MappingApplication", "logOutput_dir",
				mrpt::system::getTempFileName() + "_dir"s);
		});
}

TEST(KFSLAMApp, EKF_SLAM_2D)
{
	generic_kf_slam_test(
		"EKF-SLAM_test.ini", "kf-slam_demo.rawlog",
		[](mrpt::config::CConfigFileBase& c) {
			using namespace std::string_literals;
			c.write("MappingApplication", "SHOW_3D_LIVE", false);
			c.write("MappingApplication", "SAVE_3D_SCENES", true);
			// Output 3D scenes to a temporary directory:
			c.write(
				"MappingApplication", "logOutput_dir",
				mrpt::system::getTempFileName() + "_dir"s);
		});
}

TEST(KFSLAMApp, EKF_SLAM_3D_data_assoc_JCBB_Maha)
{
	generic_kf_slam_test(
		"EKF-SLAM_6D_test_datassoc.ini", "kf-slam_6D_demo_DA.rawlog",
		[](mrpt::config::CConfigFileBase& c) {
			using namespace std::string_literals;
			c.write("RangeBearingKFSLAM", "data_assoc_method", "assocJCBB");
			c.write("RangeBearingKFSLAM", "data_assoc_metric", "metricMaha");

			c.write("MappingApplication", "SHOW_3D_LIVE", false);
			c.write("MappingApplication", "SAVE_3D_SCENES", false);
			// Output 3D scenes to a temporary directory:
			c.write(
				"MappingApplication", "logOutput_dir",
				mrpt::system::getTempFileName() + "_dir"s);
		});
}

TEST(KFSLAMApp, EKF_SLAM_3D_data_assoc_NN_Maha)
{
	generic_kf_slam_test(
		"EKF-SLAM_6D_test_datassoc.ini", "kf-slam_6D_demo_DA.rawlog",
		[](mrpt::config::CConfigFileBase& c) {
			using namespace std::string_literals;
			c.write("RangeBearingKFSLAM", "data_assoc_method", "assocNN");
			c.write("RangeBearingKFSLAM", "data_assoc_metric", "metricMaha");

			c.write("MappingApplication", "SHOW_3D_LIVE", false);
			c.write("MappingApplication", "SAVE_3D_SCENES", false);
			// Output 3D scenes to a temporary directory:
			c.write(
				"MappingApplication", "logOutput_dir",
				mrpt::system::getTempFileName() + "_dir"s);
		});
}
