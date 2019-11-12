/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/apps/MonteCarloLocalization_App.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>
#include <iostream>

using config_changer_t = std::function<void(mrpt::config::CConfigFileBase&)>;
using post_tester_t =
	std::function<void(mrpt::apps::MonteCarloLocalization_Base&)>;

using MCL = mrpt::apps::MonteCarloLocalization_Rawlog;

void generic_pf_test(
	const std::string& ini_filename, const std::string& rawlog_filename,
	const std::string& map_filename, config_changer_t cfg_changer,
	post_tester_t post_tester)
{
	using namespace std::string_literals;

	const auto ini_fil = mrpt::UNITTEST_BASEDIR +
						 "/share/mrpt/config_files/pf-localization/"s +
						 ini_filename;
	ASSERT_FILE_EXISTS_(ini_fil);

	const auto rawlog_fil =
		mrpt::UNITTEST_BASEDIR + "/share/mrpt/datasets/"s + rawlog_filename;
	ASSERT_FILE_EXISTS_(rawlog_fil);

	const auto map_fil =
		mrpt::UNITTEST_BASEDIR + "/share/mrpt/datasets/"s + map_filename;
	ASSERT_FILE_EXISTS_(map_fil);

	try
	{
		MCL app;
		app.setMinLoggingLevel(mrpt::system::LVL_ERROR);

		const char* argv[] = {"pf-localization-slam", ini_fil.c_str(),
							  rawlog_fil.c_str()};
		const int argc = sizeof(argv) / sizeof(argv[0]);

		app.initialize(argc, argv);

		app.params.write(
			MCL::sect, "logOutput_dir",
			mrpt::system::getTempFileName() + "_dir"s);
		app.params.write(MCL::sect, "SHOW_PROGRESS_3D_REAL_TIME", false);

		app.params.write(MCL::sect, "map_file", map_fil);

		app.fill_out_estimated_path = true;
		app.allow_quit_on_esc_key = false;

#if !MRPT_HAS_OPENCV
		app.params.write(MCL::sect, "3DSceneFrequency", -1);
		app.params.write(MCL::sect, "LOG_FREQUENCY", 0);
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

static auto tester_for_localization_demo =
	[](mrpt::apps::MonteCarloLocalization_Base& o) {
		EXPECT_EQ(o.out_estimated_path.size(), 37U);
		if (o.out_estimated_path.empty()) return;

		const mrpt::math::TPose3D p = o.out_estimated_path.rbegin()->second;
		const auto p_gt = mrpt::math::TPose3D::FromString(
			"[15.89 -10.0 0.000000 4.8 0.000000 0.000000]");

		for (int i = 0; i < 6; i++) EXPECT_NEAR(p[i], p_gt[i], 0.5);
	};

TEST(MonteCarloLocalization_Rawlog, RunForSampleDataset_2D)
{
	using namespace std::string_literals;
	generic_pf_test(
		"localization_demo.ini", "localization_demo.rawlog",
		"localization_demo.simplemap.gz",
		[](mrpt::config::CConfigFileBase& cfg) {
			// Use 2D:
			cfg.write(MCL::sect, "use_3D_poses", false);
		},
		tester_for_localization_demo);
}

TEST(MonteCarloLocalization_Rawlog, RunForSampleDataset_3D)
{
	using namespace std::string_literals;
	generic_pf_test(
		"localization_demo.ini", "localization_demo.rawlog",
		"localization_demo.simplemap.gz",
		[](mrpt::config::CConfigFileBase& cfg) {
			// Use 3D:
			cfg.write(MCL::sect, "use_3D_poses", true);
			// 3D requires init in a fixed volume:
			cfg.write(MCL::sect, "init_PDF_mode", 1);
		},
		tester_for_localization_demo);
}
