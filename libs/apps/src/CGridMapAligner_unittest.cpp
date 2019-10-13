/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/apps/CGridMapAlignerApp.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

TEST(CGridMapAligner, alignGridMaps)
{
	const std::string ini_fil =
	    mrpt::UNITTEST_BASEDIR +
	    std::string(
	        "/share/mrpt/config_files/grid-matching/gridmatch_example.ini");
	EXPECT_TRUE(mrpt::system::fileExists(ini_fil));

	const std::string gridmap1_fil =
	    mrpt::UNITTEST_BASEDIR +
	    std::string("/share/mrpt/datasets/malaga-cs-fac-building.simplemap.gz");
	EXPECT_TRUE(mrpt::system::fileExists(gridmap1_fil));

	mrpt::apps::CGridMapAlignerApp app;
	app.setMinLoggingLevel(mrpt::system::LVL_ERROR);

	const char* argv[] = {"grid-matching",		"--config",
	                      ini_fil.c_str(),		"-1",
	                      gridmap1_fil.c_str(), "--detect-test"};
	const int argc = sizeof(argv) / sizeof(argv[0]);

	app.initialize(argc, argv);
	app.run();
}
