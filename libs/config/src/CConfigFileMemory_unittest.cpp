/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>
#include <fstream>
#include <cstdlib>

TEST(CConfigFileMemory, readwrite)
{
	const std::string a = "check", b = "test", c = "final //comments";
	mrpt::config::CConfigFileMemory first;
	first.write(a, b, c);
	EXPECT_STREQ("final", first.read_string(a, b, b).c_str());
}

TEST(CConfigFileMemory, Sections)
{
	std::vector<std::string> sections;
	mrpt::config::CConfigFileMemory second;
	second.write("one", "name", "val");
	second.write("two", "names", "value");
	second.getAllSections(sections);
	EXPECT_EQ(2U, sections.size());
	if (sections.size() == 2)
	{  // avoid potential crash if fails
		EXPECT_STREQ("one", sections[0].c_str());
		EXPECT_STREQ("two", sections[1].c_str());
	}
}

TEST(CConfigFileMemory, Names)
{
	std::vector<std::string> names;
	mrpt::config::CConfigFileMemory third;
	third.write("sec", "name", "val");
	third.write("sec", "names", "value");
	third.getAllKeys("sec", names);
	EXPECT_EQ(2U, names.size());
	if (names.size() == 2)
	{  // avoid potential crash if fails
		EXPECT_STREQ("name", names[0].c_str());
		EXPECT_STREQ("names", names[1].c_str());
	}
}

TEST(CConfigFileMemory, setFromString)
{
	const std::string sampleCfgTxt =
		"# example config file from std::string\n"
		"[test]\n"
		"key_num = 4\n"
		"key_str = pepe\n";

	mrpt::config::CConfigFileMemory cfg;
	cfg.setContent(sampleCfgTxt);

	EXPECT_EQ(cfg.read_int("test", "key_num", 0), 4);
	EXPECT_EQ(cfg.read_string("test", "key_str", ""), std::string("pepe"));
}

// Being able of read
const std::string sampleCfgTxt =
	"[test]\n"
	"key_str = this is a \\\n"
	"long value that can be \\\n"
	"split into several lines \\\n"
	"but read as a single line. \n";
;
const std::string expectedStr = std::string(
	"this is a long value that can be split into several lines but read as a "
	"single line.");

TEST(CConfigFileMemory, readMultiLineStrings)
{
	mrpt::config::CConfigFileMemory cfg;
	cfg.setContent(sampleCfgTxt);

	const std::string readStr = cfg.read_string("test", "key_str", "");
	EXPECT_EQ(readStr, expectedStr);
}

TEST(CConfigFile, readMultiLineStrings)
{
	const std::string tmpFile = mrpt::system::getTempFileName();
	{
		std::ofstream f;
		f.open(tmpFile.c_str(), std::ofstream::out);
		EXPECT_TRUE(f.is_open());
		f << sampleCfgTxt;
		f.close();
	}

	mrpt::config::CConfigFile cfg(tmpFile);

	const std::string readStr = cfg.read_string("test", "key_str", "");
	EXPECT_EQ(readStr, expectedStr);
}

TEST(CConfigFileMemory, parseVariables)
{
#ifdef _MSC_VER
	_putenv_s("ENV_VAR_MULTIPLIER", "2");
#else
	::setenv("ENV_VAR_MULTIPLIER", "2", 1);
#endif

	const std::string sampleCfgTxt2 =
		"@define MAXSPEED 10\n"
		"@define  MAXOMEGA  -30  \n"
		"[test]\n"
		"var1=5\n"
		"var2=${MAXSPEED}\n"
		"var3=${MAXOMEGA}\n"
		"var4=$eval{5*MAXSPEED+MAXOMEGA}\n"
		"var5 = $eval{ MAXSPEED - MAXOMEGA } \n"
		"var6=$env{ENV_VAR_MULTIPLIER}\n"
		"varstr1=MAXSPEED\n";
	;
	mrpt::config::CConfigFileMemory cfg;
	cfg.setContent(sampleCfgTxt2);

	EXPECT_EQ(cfg.read_int("test", "var1", 0), 5);
	EXPECT_EQ(cfg.read_int("test", "var2", 0), 10);
	EXPECT_EQ(cfg.read_int("test", "var3", 0), -30);
	EXPECT_NEAR(cfg.read_double("test", "var4", .0), 20.0, 1e-6);
	EXPECT_NEAR(cfg.read_double("test", "var5", .0), 40.0, 1e-6);
	EXPECT_NEAR(cfg.read_double("test", "var6", .0), 2.0, 1e-6);
	EXPECT_EQ(cfg.read_string("test", "varstr1", ""), std::string("MAXSPEED"));
}
