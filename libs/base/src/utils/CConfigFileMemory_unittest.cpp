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
	EXPECT_STREQ("one",sections[0].c_str());
	EXPECT_STREQ("two",sections[1].c_str());
	EXPECT_EQ(2,sections.size());
}

TEST(CConfigFileMemory, Names)
{
	vector_string names;
	CConfigFileMemory third;
	third.write("sec","name","val");
	third.write("sec","names","value");
	third.getAllKeys("sec", names);
	EXPECT_STREQ("name",names[0].c_str());
	EXPECT_STREQ("names",names[1].c_str());
	EXPECT_EQ(2,names.size());
}

TEST(CConfigFileMemory, multiline)
{
	const std::string a = "check", b = "test" , val = "This is a"
	" string with multiple lines"
	" used for testing";
	CConfigFileMemory fourth;
	fourth.write(a,b,val);
	EXPECT_STREQ(val.c_str(), fourth.read_string(a,b,a).c_str());
}
