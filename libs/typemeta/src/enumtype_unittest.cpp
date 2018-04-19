/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/typemeta/TEnumType.h>
#include <gtest/gtest.h>

// Example declaration of "enum class"
enum class TestColors
{
	Black = 0,
	Gray = 7,
	White = 15
};

MRPT_ENUM_TYPE_BEGIN(TestColors)
MRPT_FILL_ENUM_MEMBER(TestColors, Black);
MRPT_FILL_ENUM_MEMBER(TestColors, Gray);
MRPT_FILL_ENUM_MEMBER(TestColors, White);
MRPT_ENUM_TYPE_END()

// Example declaration of plain enum
enum Directions
{
	North,
	East,
	South,
	West
};
// Example declaration of "enum class"
MRPT_ENUM_TYPE_BEGIN(Directions)
MRPT_FILL_ENUM(North);
MRPT_FILL_ENUM(East);
MRPT_FILL_ENUM(South);
MRPT_FILL_ENUM(West);
MRPT_ENUM_TYPE_END()

TEST(TEnumType, str2value)
{
	using mrpt::typemeta::TEnumType;

	EXPECT_EQ(TEnumType<TestColors>::name2value("White"), TestColors::White);
	EXPECT_EQ(TEnumType<TestColors>::name2value("Black"), TestColors::Black);
	EXPECT_EQ(TEnumType<TestColors>::name2value("Gray"), TestColors::Gray);

	EXPECT_EQ(TEnumType<Directions>::name2value("East"), East);

	EXPECT_THROW(TEnumType<TestColors>::name2value("Violet"), std::exception);
}

TEST(TEnumType, value2str)
{
	using mrpt::typemeta::TEnumType;

	EXPECT_EQ(TEnumType<TestColors>::value2name(TestColors::White), "White");
	EXPECT_EQ(TEnumType<TestColors>::value2name(TestColors::Black), "Black");
	EXPECT_EQ(TEnumType<TestColors>::value2name(TestColors::Gray), "Gray");

	EXPECT_EQ(TEnumType<Directions>::value2name(East), "East");

	EXPECT_THROW(
		TEnumType<TestColors>::value2name(static_cast<TestColors>(5)),
		std::exception);
}
