/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/apps/RBPF_SLAM_App.h>
#include <mrpt/poses/Lie/SE.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>
#include <iostream>

using config_changer_t = std::function<void(mrpt::config::CConfigFileBase&)>;
using post_tester_t = std::function<void(mrpt::apps::RBPF_SLAM_App_Base&)>;

void generic_rbpf_slam_test(
	const std::string& ini_filename, const std::string& rawlog_filename,
	config_changer_t cfg_changer, post_tester_t post_tester)
{
	using namespace std::string_literals;

	const auto ini_fil = mrpt::UNITTEST_BASEDIR +
						 "/share/mrpt/config_files/rbpf-slam/"s + ini_filename;
	EXPECT_TRUE(mrpt::system::fileExists(ini_fil));

	const auto rawlog_fil =
		mrpt::UNITTEST_BASEDIR + "/share/mrpt/datasets/"s + rawlog_filename;
	EXPECT_TRUE(mrpt::system::fileExists(rawlog_fil));

	try
	{
		mrpt::apps::RBPF_SLAM_App_Rawlog app;
		app.setMinLoggingLevel(mrpt::system::LVL_WARN);

		const char* argv[] = {"rbpf-slam", ini_fil.c_str(), rawlog_fil.c_str()};
		const int argc = sizeof(argv) / sizeof(argv[0]);

		app.initialize(argc, argv);

		app.params.write(
			"MappingApplication", "logOutput_dir",
			mrpt::system::getTempFileName() + "_dir"s);
		app.params.write(
			"MappingApplication", "SHOW_PROGRESS_IN_WINDOW", false);

#if !MRPT_HAS_OPENCV
		app.params.write("MappingApplication", "SAVE_3D_SCENE", false);
		app.params.write("MappingApplication", "LOG_FREQUENCY", 0);
#endif

		cfg_changer(app.params);
		app.run();

		// Check results:
		post_tester(app);
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		GTEST_FAIL();
	}
}

static auto tester_for_2006_01_21 = [](mrpt::apps::RBPF_SLAM_App_Base& o) {
	EXPECT_EQ(o.out_estimated_path.size(), 224U);
	const auto p = mrpt::poses::CPose3D(o.out_estimated_path.rbegin()->second);
	const auto p_gt = mrpt::poses::CPose3D::FromString(
		"[3.4548 -18.0399 0.000000 -86.48 0.000000 0.000000]");

	EXPECT_LT(mrpt::poses::Lie::SE<3>::log(p - p_gt).norm(), 1.0)
		<< "actual pose  =" << p.asString()
		<< "\nexpected pose=" << p_gt.asString();
};

static auto tester_for_ROSLAM_demo = [](mrpt::apps::RBPF_SLAM_App_Base& o) {
	EXPECT_EQ(o.out_estimated_path.size(), 99U);
	const auto p = mrpt::poses::CPose3D(o.out_estimated_path.rbegin()->second);
	const auto p_gt = mrpt::poses::CPose3D::FromString(
		"[1.938686 3.352273 0.000000 114.993417 0.000000 0.000000]");

	EXPECT_LT(mrpt::poses::Lie::SE<3>::log(p - p_gt).norm(), 1.0)
		<< "actual pose  =" << p.asString()
		<< "\nexpected pose=" << p_gt.asString();

	MRPT_TODO("Stricter unit tests: check for estimated landmark positions");
};

TEST(RBPF_SLAM_App, MapFromRawlog_Lidar2D_optimal_sampling)
{
	using namespace std::string_literals;

	generic_rbpf_slam_test(
		"gridmapping_optimal_sampling.ini",
		"2006-01ENE-21-SENA_Telecom Faculty_one_loop_only.rawlog",
		[](mrpt::config::CConfigFileBase&) {}, tester_for_2006_01_21);
}

TEST(RBPF_SLAM_App, MapFromRawlog_Lidar2D_gridICP)
{
	using namespace std::string_literals;

	generic_rbpf_slam_test(
		"gridmapping_RBPF_grid_ICPbased_malaga.ini",
		"2006-01ENE-21-SENA_Telecom Faculty_one_loop_only.rawlog",
		[](mrpt::config::CConfigFileBase&) {}, tester_for_2006_01_21);
}

TEST(RBPF_SLAM_App, MapFromRawlog_Lidar2D_pointsICP)
{
	using namespace std::string_literals;

	generic_rbpf_slam_test(
		"gridmapping_RBPF_ICPbased_malaga.ini",
		"2006-01ENE-21-SENA_Telecom Faculty_one_loop_only.rawlog",
		[](mrpt::config::CConfigFileBase&) {}, tester_for_2006_01_21);
}

TEST(RBPF_SLAM_App, MapFromRawlog_ROSLAM_MC)
{
	using namespace std::string_literals;

	generic_rbpf_slam_test(
		"RO-SLAM_simulatedData_MC.ini", "RO-SLAM_demo.rawlog",
		[](mrpt::config::CConfigFileBase&) {}, tester_for_ROSLAM_demo);
}

TEST(RBPF_SLAM_App, MapFromRawlog_ROSLAM_SOG)
{
	using namespace std::string_literals;

	generic_rbpf_slam_test(
		"RO-SLAM_simulatedData_SOG.ini", "RO-SLAM_demo.rawlog",
		[](mrpt::config::CConfigFileBase&) {}, tester_for_ROSLAM_demo);
}
