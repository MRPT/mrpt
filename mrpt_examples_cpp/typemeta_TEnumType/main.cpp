/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/
/** \example typemeta_TEnumType/main.cpp */

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

  std::cout << "White => " << (int)TEnumType<TestColors>::name2value("White") << "\n";
  std::cout << "Black => " << (int)TEnumType<TestColors>::name2value("Black") << "\n";
  std::cout << "Gray  => " << (int)TEnumType<TestColors>::name2value("Gray") << "\n";

  std::cout << "7    <= " << TEnumType<TestColors>::value2name(TestColors(7)) << "\n";
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
    std::cerr << "MRPT error: " << e.what() << "\n";
    return -1;
  }
}
