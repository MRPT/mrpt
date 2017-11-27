/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/typemeta/TEnumType.h>
#include <iostream>

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
	static void fill(mrpt::typemeta::internal::bimap<enum_t, std::string>& m_map)
	{
		m_map.insert(TestColors::Black, "Black");
		m_map.insert(TestColors::Gray, "Gray");
		m_map.insert(TestColors::White, "White");
	}
};

void Test_EnumType()
{
	using namespace std;
	using namespace mrpt::typemeta;

	cout << "White => " << (int)TEnumType<TestColors>::name2value("White") << endl;
	cout << "Black => " << (int)TEnumType<TestColors>::name2value("Black") << endl;
	cout << "Gray  => " << (int)TEnumType<TestColors>::name2value("Gray") << endl;

	cout << "7    <= " << TEnumType<TestColors>::value2name(TestColors(7)) << endl;
}

int main(int argc, char** argv)
{
	try
	{
		Test_EnumType();
		return 0;
	}
	catch (std::exception& e)
	{
		std::cout << "Exception: " << e.what() << std::endl;
		return -1;
	}
}
