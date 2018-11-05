/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
/** \example typemeta_TEnumType/test.cpp */

//! [example]
#include <mrpt/typemeta/TEnumType.h>
#include <iostream>
#include <string>

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

void Test_EnumType()
{
	using namespace std;
	using namespace mrpt::typemeta;

	cout << "White => " << (int)TEnumType<TestColors>::name2value("White")
		 << endl;
	cout << "Black => " << (int)TEnumType<TestColors>::name2value("Black")
		 << endl;
	cout << "Gray  => " << (int)TEnumType<TestColors>::name2value("Gray")
		 << endl;

	cout << "7    <= " << TEnumType<TestColors>::value2name(TestColors(7))
		 << endl;
}
//! [example]

int main(int argc, char** argv)
{
	try
	{
		Test_EnumType();
		return 0;
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
}
