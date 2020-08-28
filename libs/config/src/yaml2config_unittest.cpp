/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/config.h>

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>

// clang-format off
const auto sampleYamlTxt = std::string(R"xxx(
# example YAML config file from a std::string
unscoped_key1: 42
maps:
  pointMaps: 1
  gridMaps: 2
pointMap_00:
  max_x: 10.0
  min_x: -10.0
gridmap_00: {max_x: 25.0, min_x: -25.0}
gridmap_01:
  foo1: 0
  layerName: lidar
# end of YAMLexample
)xxx");
// clang-format on

const std::string sampleCfgTxt =
	"# example config file from std::string\n"
	"[test]\n"
	"key_num = 4\n"
	"key_str = pepe\n";

const std::string sampleCfgTxt_as_yaml_correct =
	"test:\n"
	"  key_num: 4\n"
	"  key_str: pepe\n";

TEST(yaml2ini, parse)
{
	try
	{
		mrpt::config::CConfigFileMemory c;
		c.setContentFromYAML(sampleYamlTxt);

		// std::cout << "INI-liKe:\n" << c.getContent() << "\n";

		EXPECT_EQ(1, c.read_int("maps", "pointMaps", 0));
		EXPECT_EQ(2, c.read_int("maps", "gridMaps", 0));
		EXPECT_EQ(25.0, c.read_int("gridmap_00", "max_x", 0));
		EXPECT_EQ(42, c.read_int("", "unscoped_key1", 0));
	}
	catch (const std::exception& e)
	{
		std::cerr << mrpt::exception_to_str(e);
		GTEST_FAIL();
	}
}

TEST(ini2yaml, parse)
{
	mrpt::config::CConfigFileMemory c;
	c.setContent(sampleCfgTxt);

	// Note: we don't compare the exact strings since order of keys may vary (?)
	EXPECT_EQ(sampleCfgTxt_as_yaml_correct.size(), c.getContentAsYAML().size());
}
