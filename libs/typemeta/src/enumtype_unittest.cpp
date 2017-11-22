/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/typemeta/TEnumType.h>
#include <gtest/gtest.h>

enum class TestColors
{
	Black = 0,
	Gray = 7,
	White = 15
};

template <>
struct mrpt::typemeta::TEnumTypeFiller<TestColors>
{
	using enum_t = TestColors;
	static void fill(internal::bimap<enum_t, std::string>& m_map)
	{
		m_map.insert(TestColors::Black, "Black");
		m_map.insert(TestColors::Gray, "Gray");
		m_map.insert(TestColors::White, "White");
	}
};


TEST(TEnumType, str2value)
{
	EXPECT_EQ(mrpt::typemeta::TEnumType<TestColors>::name2value("White"), TestColors::White);
	EXPECT_EQ(mrpt::typemeta::TEnumType<TestColors>::name2value("Black"), TestColors::Black);
	EXPECT_EQ(mrpt::typemeta::TEnumType<TestColors>::name2value("Gray"), TestColors::Gray);
	
	try
	{
		auto v = mrpt::typemeta::TEnumType<TestColors>::name2value("Violet");
		EXPECT_FALSE(true) << "Expected exception but it didn't happen!";
	}
	catch (std::exception &)
	{
		// All good
	}
}
