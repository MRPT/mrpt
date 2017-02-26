/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CConfigFileMemory.h>
#include <gtest/gtest.h>

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
	EXPECT_EQ(2, sections.size());
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
	EXPECT_EQ(2, names.size());
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

TEST(CConfigFileMemory, readMultiLineStrings)
{
	// Being able of read 
	const std::string sampleCfgTxt =
		"[test]\n"
		"key_str = this is a \\\n"
		"long value that can be \\\n"
		"split into several lines\\\n"
		"but read as a single line. \n";
		;

	CConfigFileMemory cfg;
	cfg.setContent(sampleCfgTxt);

	// ..
	EXPECT_EQ(cfg.read_string("test","key_str",""),std::string("this is a long value that can be split into several linesbut read as a single line."));	
}
