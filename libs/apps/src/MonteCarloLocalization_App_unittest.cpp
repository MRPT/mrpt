/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/apps/MonteCarloLocalization_App.h>
#include <mrpt/poses/Lie/SE.h>
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

static bool tester_result_ok = true;

static auto tester_for_localization_demo =
	[](mrpt::apps::MonteCarloLocalization_Base& o) {
		EXPECT_EQ(o.out_estimated_path.size(), 37U);
		if (o.out_estimated_path.empty()) return;

		const mrpt::math::TPose3D p = o.out_estimated_path.rbegin()->second;
		const auto p_gt = mrpt::math::TPose3D::FromString(
			"[15.89 -10.0 0.000000 4.8 0.000000 0.000000]");

		const auto p_err = mrpt::poses::CPose3D(p_gt - p);
		const double err = mrpt::poses::Lie::SE<3>::log(p_err).norm();

		if (err < 0.5)
		{
			tester_result_ok = true;
		}
		else
		{
			tester_result_ok = false;
			std::cerr << "Final pose mismatch (will retry N times):\n"
						 "Expected: "
					  << p_gt.asString()
					  << "\n"
						 "Actual  : "
					  << p.asString() << "\n";
		}
	};

TEST(MonteCarloLocalization_Rawlog, RunForSampleDataset_2D)
{
	using namespace std::string_literals;
	for (int tries = 0; tries < 5; tries++)
	{
		generic_pf_test(
			"localization_demo.ini", "localization_demo.rawlog",
			"localization_demo.simplemap.gz",
			[](mrpt::config::CConfigFileBase& cfg) {
				// Use 2D:
				cfg.write(MCL::sect, "use_3D_poses", false);
			},
			tester_for_localization_demo);

		if (tester_result_ok) break;
	}
}

TEST(MonteCarloLocalization_Rawlog, RunForSampleDataset_3D)
{
	using namespace std::string_literals;
	for (int tries = 0; tries < 5; tries++)
	{
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

		if (tester_result_ok) break;
	}
}
