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
/** \example typemeta_StaticString/main.cpp */

//! [example sstring]
#include <mrpt/typemeta/static_string.h>

#include <iostream>

void Test_StaticString()
{
  using namespace std;
  using namespace mrpt::typemeta;

  constexpr string_literal<3> s = "foo";
  std::cout << "string_literal<3>=" << s << "\n";

  constexpr auto a = literal("foo");
  constexpr auto b = literal("bar");
  // In GCC7 this can be "constexpr", but it fails in MSVC 2017 (!)
  auto ab = a + b;
  std::cout << "a=" << a << "\n";
  std::cout << "b=" << b << "\n";
  std::cout << "a+b=" << ab << "\n";

  static_assert(ab.size() == 6, "***");
  std::cout << "(a+b).size()=" << ab.size() << "\n";
  std::cout << "(a+b)[0]=" << ab[0] << "\n";
  std::cout << "(a+b)[5]=" << ab[5] << "\n";
}
//! [example sstring]

//! [example num2str]
#include <mrpt/typemeta/num_to_string.h>

#include <iostream>

void Test_StaticNum2Str()
{
  using namespace std;
  using namespace mrpt::typemeta;

  constexpr auto n42 = num_to_string<42>::value;
  constexpr auto n9999 = num_to_string<9999>::value;
  std::cout << "42 as string=" << n42 << "\n";
  std::cout << "9999 as string=" << n9999 << "\n";
}
//! [example num2str]

int main(int argc, char** argv)
{
  try
  {
    Test_StaticString();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << e.what() << "\n";
    return -1;
  }
}
