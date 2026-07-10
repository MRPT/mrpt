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

#include <gtest/gtest.h>
#include <mrpt/img/TPixelCoord.h>

using namespace mrpt::img;

TEST(TPixelCoord, ConstructAndCompare)
{
  const TPixelCoord a(3, 4);
  const TPixelCoord b(3, 4);
  const TPixelCoord c(1, 2);

  EXPECT_TRUE(a == b);
  EXPECT_FALSE(a == c);
}

TEST(TPixelCoord, ArithmeticOperators)
{
  const TPixelCoord a(3, 4);
  const TPixelCoord b(1, 2);

  EXPECT_TRUE((a + b) == TPixelCoord(4, 6));
  EXPECT_TRUE((a - b) == TPixelCoord(2, 2));
}

TEST(TPixelCoord, ConvertingCtor)
{
  const TPixelCoordf f(1.9f, 2.1f);
  const TPixelCoord i(f);
  EXPECT_EQ(i.x, 1);
  EXPECT_EQ(i.y, 2);
}

TEST(TPixelCoord, TextStreaming)
{
  const TPixelCoord p(7, -3);
  std::ostringstream ss;
  ss << p;
  EXPECT_EQ(ss.str(), "(7,-3)");

  const TPixelCoordf pf(1.5f, 2.5f);
  std::ostringstream ssf;
  ssf << pf;
  EXPECT_EQ(ssf.str(), "(1.5,2.5)");
}

TEST(TPixelCoord, DefaultCtorIsZero)
{
  const TPixelCoord p;
  EXPECT_EQ(p.x, 0);
  EXPECT_EQ(p.y, 0);
}
