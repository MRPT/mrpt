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
#include <mrpt/imgui/style.h>

using namespace mrpt::imgui;

TEST(style, ColorConversions)
{
  const mrpt::img::TColor c(10, 128, 255, 40);
  const ImVec4 v = toImVec4(c);
  EXPECT_NEAR(v.x, 10 / 255.0f, 1e-6);
  EXPECT_NEAR(v.w, 40 / 255.0f, 1e-6);
  const auto back = toTColor(v);
  EXPECT_EQ(back.R, c.R);
  EXPECT_EQ(back.G, c.G);
  EXPECT_EQ(back.B, c.B);
  EXPECT_EQ(back.A, c.A);

  const auto clamped = toTColor(ImVec4(-1.0f, 2.0f, 0.5f, 1.0f));
  EXPECT_EQ(clamped.R, 0);
  EXPECT_EQ(clamped.G, 255);
  EXPECT_EQ(clamped.B, 128);

  EXPECT_EQ(toImU32(mrpt::img::TColor(1, 2, 3, 4)), IM_COL32(1, 2, 3, 4));
  const ImVec4 vf = toImVec4(mrpt::img::TColorf(0.1f, 0.2f, 0.3f, 0.4f));
  EXPECT_FLOAT_EQ(vf.z, 0.3f);
}
