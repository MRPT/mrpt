/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

/** \example core_exceptions_example/test.cpp */

//! [example-nested-exceptions]

#include <mrpt/core/exceptions.h>

#include <iostream>

void test_except_3rd_lvl()
{
  MRPT_START
  THROW_EXCEPTION("Aw!");
  MRPT_END
}

void test_except_2nd_lvl()
{
  MRPT_START
  test_except_3rd_lvl();
  MRPT_END
}

void test_except_toplevel()
{
  MRPT_START
  test_except_2nd_lvl();
  MRPT_END
}

void test_except_2nd_lvl_bis()
{
  MRPT_START
  std::vector<int> x;
  x.resize(2);
  x.at(10);  // throws
  MRPT_END
}

void test_except_toplevel_bis() { test_except_2nd_lvl_bis(); }

int main()
{
  try
  {
    test_except_toplevel();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what();
  }

  try
  {
    test_except_toplevel_bis();
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what();
  }

  return 0;
}
//! [example-nested-exceptions]
