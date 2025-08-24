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

#include <gtest/gtest.h>
#include <mrpt/img/color_maps.h>

TEST(color_maps, cmGRAYSCALE)
{
  using namespace mrpt::img;

  {
    float r, g, b;
    colormap(cmGRAYSCALE, .0f, r, g, b);
    EXPECT_NEAR(r, .0f, 1e-3f);
    EXPECT_NEAR(g, .0f, 1e-3f);
    EXPECT_NEAR(b, .0f, 1e-3f);
  }
  {
    float r, g, b;
    colormap(cmGRAYSCALE, 1.0f, r, g, b);
    EXPECT_NEAR(r, 1.0f, 1e-3f);
    EXPECT_NEAR(g, 1.0f, 1e-3f);
    EXPECT_NEAR(b, 1.0f, 1e-3f);
  }
  {
    float r, g, b;
    colormap(cmGRAYSCALE, -0.1f, r, g, b);
    EXPECT_NEAR(r, .0f, 1e-3f);
    EXPECT_NEAR(g, .0f, 1e-3f);
    EXPECT_NEAR(b, .0f, 1e-3f);
  }
  {
    float r, g, b;
    colormap(cmGRAYSCALE, 1.2f, r, g, b);
    EXPECT_NEAR(r, 1.0f, 1e-3f);
    EXPECT_NEAR(g, 1.0f, 1e-3f);
    EXPECT_NEAR(b, 1.0f, 1e-3f);
  }
  {
    float r, g, b;
    colormap(cmGRAYSCALE, 0.8f, r, g, b);
    EXPECT_NEAR(r, 0.8f, 1e-3f);
    EXPECT_NEAR(g, 0.8f, 1e-3f);
    EXPECT_NEAR(b, 0.8f, 1e-3f);
  }
}

TEST(color_maps, cmJET)
{
  using namespace mrpt::img;

  {
    float r, g, b;
    colormap(cmJET, .0f, r, g, b);
    EXPECT_NEAR(r, 0.0f, 1e-3f);
    EXPECT_NEAR(g, 0.0f, 1e-3f);
    EXPECT_NEAR(b, 0.5625f, 1e-3f);
  }
  {
    float r, g, b;
    colormap(cmJET, 1.0f, r, g, b);
    EXPECT_NEAR(r, 0.5f, 1e-3f);
    EXPECT_NEAR(g, 0.0f, 1e-3f);
    EXPECT_NEAR(b, 0.0f, 1e-3f);
  }
  {
    float r, g, b;
    colormap(cmJET, 0.8f, r, g, b);
    EXPECT_NEAR(r, 1.0f, 1e-3f);
    EXPECT_NEAR(g, 0.2375f, 1e-3f);
    EXPECT_NEAR(b, 0.0f, 1e-3f);
  }
}

TEST(color_maps, cmHOT)
{
  using namespace mrpt::img;

  {
    float r, g, b;
    colormap(cmHOT, .0f, r, g, b);
    EXPECT_NEAR(r, 0.04166f, 1e-3f);
    EXPECT_NEAR(g, 0.0f, 1e-3f);
    EXPECT_NEAR(b, 0.0f, 1e-3f);
  }
  {
    float r, g, b;
    colormap(cmHOT, 1.0f, r, g, b);
    EXPECT_NEAR(r, 1.0f, 1e-3f);
    EXPECT_NEAR(g, 1.0f, 1e-3f);
    EXPECT_NEAR(b, 1.0f, 1e-3f);
  }
  {
    float r, g, b;
    colormap(cmHOT, 0.8f, r, g, b);
    EXPECT_NEAR(r, 1.0f, 1e-3f);
    EXPECT_NEAR(g, 1.0f, 1e-3f);
    EXPECT_NEAR(b, 0.2625f, 1e-3f);
  }
}
