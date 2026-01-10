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
#include <mrpt/img/color_maps.h>

TEST(color_maps, cmGRAYSCALE)
{
  using namespace mrpt::img;

  {
    const auto c = colormap(cmGRAYSCALE, .0f);
    EXPECT_NEAR(c.R, .0f, 1e-3f);
    EXPECT_NEAR(c.G, .0f, 1e-3f);
    EXPECT_NEAR(c.B, .0f, 1e-3f);
  }
  {
    const auto c = colormap(cmGRAYSCALE, 1.0f);
    EXPECT_NEAR(c.R, 1.0f, 1e-3f);
    EXPECT_NEAR(c.G, 1.0f, 1e-3f);
    EXPECT_NEAR(c.B, 1.0f, 1e-3f);
  }
  {
    const auto c = colormap(cmGRAYSCALE, -0.1f);
    EXPECT_NEAR(c.R, .0f, 1e-3f);
    EXPECT_NEAR(c.G, .0f, 1e-3f);
    EXPECT_NEAR(c.B, .0f, 1e-3f);
  }
  {
    const auto c = colormap(cmGRAYSCALE, 1.2f);
    EXPECT_NEAR(c.R, 1.0f, 1e-3f);
    EXPECT_NEAR(c.G, 1.0f, 1e-3f);
    EXPECT_NEAR(c.B, 1.0f, 1e-3f);
  }
  {
    const auto c = colormap(cmGRAYSCALE, 0.8f);
    EXPECT_NEAR(c.R, 0.8f, 1e-3f);
    EXPECT_NEAR(c.G, 0.8f, 1e-3f);
    EXPECT_NEAR(c.B, 0.8f, 1e-3f);
  }
}

TEST(color_maps, cmJET)
{
  using namespace mrpt::img;

  {
    const auto c = colormap(cmJET, .0f);
    EXPECT_NEAR(c.R, 0.0f, 1e-3f);
    EXPECT_NEAR(c.G, 0.0f, 1e-3f);
    EXPECT_NEAR(c.B, 0.5625f, 1e-3f);
  }
  {
    const auto c = colormap(cmJET, 1.0f);
    EXPECT_NEAR(c.R, 0.5f, 1e-3f);
    EXPECT_NEAR(c.G, 0.0f, 1e-3f);
    EXPECT_NEAR(c.B, 0.0f, 1e-3f);
  }
  {
    const auto c = colormap(cmJET, 0.8f);
    EXPECT_NEAR(c.R, 1.0f, 1e-3f);
    EXPECT_NEAR(c.G, 0.2375f, 1e-3f);
    EXPECT_NEAR(c.B, 0.0f, 1e-3f);
  }
}

TEST(color_maps, cmHOT)
{
  using namespace mrpt::img;

  {
    const auto c = colormap(cmHOT, .0f);
    EXPECT_NEAR(c.R, 0.04166f, 1e-3f);
    EXPECT_NEAR(c.G, 0.0f, 1e-3f);
    EXPECT_NEAR(c.B, 0.0f, 1e-3f);
  }
  {
    const auto c = colormap(cmHOT, 1.0f);
    EXPECT_NEAR(c.R, 1.0f, 1e-3f);
    EXPECT_NEAR(c.G, 1.0f, 1e-3f);
    EXPECT_NEAR(c.B, 1.0f, 1e-3f);
  }
  {
    const auto c = colormap(cmHOT, 0.8f);
    EXPECT_NEAR(c.R, 1.0f, 1e-3f);
    EXPECT_NEAR(c.G, 1.0f, 1e-3f);
    EXPECT_NEAR(c.B, 0.2625f, 1e-3f);
  }
}
