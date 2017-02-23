/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/system/os.h>

#include "simpleini/SimpleIni.h"

#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::utils::simpleini;
using namespace std;

class TConfig : public CConfigFileMemory {
	public:
	using CConfigFileMemory::writeString;
	using CConfigFileMemory::readString;
	using CConfigFileMemory::getAllSections;
	using CConfigFileMemory::getAllKeys;
	};

TEST(CConfigFileMemory, readwrite)
{
	const std::string a = "check", b = "test", c = "final //comment", d = "final";
	TConfig second;
	second.writeString(a,b,c);
	EXPECT_STREQ("final ", second.readString(a,b,d).c_str());	
}

TEST(CConfigFileMemory, Sections)
{
	vector_string sections;
	TConfig third;
	third.writeString("one","name","val");
	third.writeString("two","names","value");
	third.getAllSections(sections);
	EXPECT_STREQ("one",sections[0].c_str());
	EXPECT_STREQ("two",sections[1].c_str());
	EXPECT_EQ(2,sections.size());
}

TEST(CConfigFileMemory, Names)
{
	vector_string names;
	TConfig fourth;
	fourth.writeString("sec","name","val");
	fourth.writeString("sec","names","value");
	fourth.getAllKeys("sec", names);
	EXPECT_STREQ("name",names[0].c_str());
	EXPECT_STREQ("names",names[1].c_str());
	EXPECT_EQ(2,names.size());
}
