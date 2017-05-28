/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>
#include <fstream>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

TEST(CConfigFileMemory, readwrite)
{
	const std::string a = "check", b = "test", c = "final //comments";
	CConfigFileMemory first;
	first.write(a,b,c);
	EXPECT_STREQ("final", first.read_string(a,b,b).c_str());	
}

TEST(CConfigFileMemory, Sections)
{
	vector_string sections;
	CConfigFileMemory second;
	second.write("one","name","val");
	second.write("two","names","value");
	second.getAllSections(sections);
	EXPECT_EQ(2U, sections.size());
	if (sections.size() == 2) {  // avoid potential crash if fails
		EXPECT_STREQ("one", sections[0].c_str());
		EXPECT_STREQ("two", sections[1].c_str());
	}
}

TEST(CConfigFileMemory, Names)
{
	vector_string names;
	CConfigFileMemory third;
	third.write("sec","name","val");
	third.write("sec","names","value");
	third.getAllKeys("sec", names);
	EXPECT_EQ(2U, names.size());
	if (names.size() == 2) {  // avoid potential crash if fails
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
		"key_str = pepe\n"
		;

	CConfigFileMemory cfg;
	cfg.setContent(sampleCfgTxt);

	EXPECT_EQ(cfg.read_int("test", "key_num", 0), 4);
	EXPECT_EQ(cfg.read_string("test","key_str",""),std::string("pepe"));
}

// Being able of read 
const std::string sampleCfgTxt =
"[test]\n"
"key_str = this is a \\\n"
"long value that can be \\\n"
"split into several lines \\\n"
"but read as a single line. \n";
;
const std::string expectedStr = std::string("this is a long value that can be split into several lines but read as a single line.");

TEST(CConfigFileMemory, readMultiLineStrings)
{
	CConfigFileMemory cfg;
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

	CConfigFile cfg(tmpFile);

	const std::string readStr = cfg.read_string("test", "key_str", "");
	EXPECT_EQ(readStr, expectedStr);
}

TEST(CConfigFileMemory, parseVariables)
{
	const std::string sampleCfgTxt2 =
		"@define MAXSPEED 10\n"
		"@define  MAXOMEGA  -30  \n"
		"[test]\n"
		"var1=5\n"
		"var2=${MAXSPEED}\n"
		"var3=${MAXOMEGA}\n"
		"var4=$eval{5*MAXSPEED+MAXOMEGA}\n"
		"var5 = $eval{ MAXSPEED - MAXOMEGA } \n"
		"varstr1=MAXSPEED\n";
	;
	CConfigFileMemory cfg;
	cfg.setContent(sampleCfgTxt2);

	EXPECT_EQ(cfg.read_int("test", "var1", 0), 5);
	EXPECT_EQ(cfg.read_int("test", "var2", 0), 10);
	EXPECT_EQ(cfg.read_int("test", "var3", 0), -30);
	EXPECT_NEAR(cfg.read_double("test", "var4", .0), 20.0, 1e-6);
	EXPECT_NEAR(cfg.read_double("test", "var5", .0), 40.0,1e-6);
	EXPECT_EQ(cfg.read_string("test", "varstr1", ""), std::string("MAXSPEED"));
}
