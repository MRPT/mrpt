/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/apps/CGridMapAlignerApp.h>
#include <mrpt/config.h>
#include <mrpt/math/TPose2D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

#if MRPT_HAS_OPENCV
TEST(CGridMapAligner, alignGridMaps)
#else
TEST(CGridMapAligner, DISABLED_alignGridMaps)
#endif
{
	const std::string ini_fil =
		mrpt::UNITTEST_BASEDIR +
		std::string(
			"/share/mrpt/config_files/grid-matching/gridmatch_example.ini");
	EXPECT_TRUE(mrpt::system::fileExists(ini_fil));

	const std::string gridmap1_fil = mrpt::UNITTEST_BASEDIR +
		std::string("/share/mrpt/datasets/malaga-cs-fac-building.simplemap.gz");
	EXPECT_TRUE(mrpt::system::fileExists(gridmap1_fil));

	try
	{
		// Since this uses RANSAC, there is a small chance of random failure:
		for (int tries = 0; tries < 3; tries++)
		{
			mrpt::apps::CGridMapAlignerApp app;
			app.setMinLoggingLevel(mrpt::system::LVL_ERROR);

			const char* argv[] = {
				"grid-matching", "--config",		   ini_fil.c_str(),
				"--map1",		 gridmap1_fil.c_str(), "--detect-test",
				"--nologo",		 "--nosave",		   "--noicp"};
			const int argc = sizeof(argv) / sizeof(argv[0]);

			app.initialize(argc, argv);
			app.run();

			// Check result:
			if (std::abs(app.estimateMean.x() - app.GT_Ax) < 0.1 &&
				std::abs(app.estimateMean.y() - app.GT_Ay) < 0.1 &&
				mrpt::math::angDistance(
					app.estimateMean.phi(), app.GT_Aphi_rad) < 0.05)
			{
				// good result.
				return;
			}
			std::cout << "Attempt #" << tries
					  << " failed:\n"
						 "Estimation: "
					  << app.estimateMean.asString()
					  << "\n"
						 "GT:         "
					  << mrpt::math::TPose2D(
							 app.GT_Ax, app.GT_Ay, app.GT_Aphi_rad)
							 .asString();
		}
		GTEST_FAIL() << "Ground truth not reached after several attempts.";
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		GTEST_FAIL();
	}
}
