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

TEST(color_maps, colormap_InvalidThrows)
{
  using namespace mrpt::img;
  EXPECT_THROW(colormap(cmNONE, 0.5f), std::exception);
}

TEST(color_maps, hsv2rgb_PrimaryAndSecondaryColors)
{
  using namespace mrpt::img;

  // Red (h=0)
  {
    const auto c = hsv2rgb(0.0f, 1.0f, 1.0f);
    EXPECT_NEAR(c.R, 1.0f, 1e-3f);
    EXPECT_NEAR(c.G, 0.0f, 1e-3f);
    EXPECT_NEAR(c.B, 0.0f, 1e-3f);
  }
  // Yellow (h=1/6)
  {
    const auto c = hsv2rgb(1.0f / 6.0f, 1.0f, 1.0f);
    EXPECT_NEAR(c.R, 1.0f, 1e-2f);
    EXPECT_NEAR(c.G, 1.0f, 1e-2f);
    EXPECT_NEAR(c.B, 0.0f, 1e-2f);
  }
  // Green (h=2/6)
  {
    const auto c = hsv2rgb(2.0f / 6.0f, 1.0f, 1.0f);
    EXPECT_NEAR(c.R, 0.0f, 1e-2f);
    EXPECT_NEAR(c.G, 1.0f, 1e-2f);
    EXPECT_NEAR(c.B, 0.0f, 1e-2f);
  }
  // Cyan (h=3/6)
  {
    const auto c = hsv2rgb(3.0f / 6.0f, 1.0f, 1.0f);
    EXPECT_NEAR(c.R, 0.0f, 1e-2f);
    EXPECT_NEAR(c.G, 1.0f, 1e-2f);
    EXPECT_NEAR(c.B, 1.0f, 1e-2f);
  }
  // Blue (h=4/6)
  {
    const auto c = hsv2rgb(4.0f / 6.0f, 1.0f, 1.0f);
    EXPECT_NEAR(c.R, 0.0f, 1e-2f);
    EXPECT_NEAR(c.G, 0.0f, 1e-2f);
    EXPECT_NEAR(c.B, 1.0f, 1e-2f);
  }
  // Magenta (h=5/6)
  {
    const auto c = hsv2rgb(5.0f / 6.0f, 1.0f, 1.0f);
    EXPECT_NEAR(c.R, 1.0f, 1e-2f);
    EXPECT_NEAR(c.G, 0.0f, 1e-2f);
    EXPECT_NEAR(c.B, 1.0f, 1e-2f);
  }
}

TEST(color_maps, hsv2rgb_ZeroSaturationIsGray)
{
  using namespace mrpt::img;
  const auto c = hsv2rgb(0.3f, 0.0f, 0.6f);
  EXPECT_NEAR(c.R, 0.6f, 1e-3f);
  EXPECT_NEAR(c.G, 0.6f, 1e-3f);
  EXPECT_NEAR(c.B, 0.6f, 1e-3f);
}

TEST(color_maps, hsv2rgb_ClampsOutOfRangeInputs)
{
  using namespace mrpt::img;
  const auto c1 = hsv2rgb(-1.0f, 2.0f, -5.0f);
  const auto c2 = hsv2rgb(0.0f, 1.0f, 0.0f);
  EXPECT_NEAR(c1.R, c2.R, 1e-3f);
  EXPECT_NEAR(c1.G, c2.G, 1e-3f);
  EXPECT_NEAR(c1.B, c2.B, 1e-3f);
}

TEST(color_maps, rgb2hsv_PrimaryColors)
{
  using namespace mrpt::img;

  {
    const auto [h, s, v] = rgb2hsv(1.0f, 0.0f, 0.0f);
    EXPECT_NEAR(h, 0.0f, 1e-3f);
    EXPECT_NEAR(s, 1.0f, 1e-3f);
    EXPECT_NEAR(v, 1.0f, 1e-3f);
  }
  {
    // Max == G branch
    const auto [h, s, v] = rgb2hsv(0.0f, 1.0f, 0.0f);
    EXPECT_NEAR(h, 1.0f / 3.0f, 1e-3f);
    EXPECT_NEAR(s, 1.0f, 1e-3f);
    EXPECT_NEAR(v, 1.0f, 1e-3f);
  }
  {
    // Max == B branch
    const auto [h, s, v] = rgb2hsv(0.0f, 0.0f, 1.0f);
    EXPECT_NEAR(h, 2.0f / 3.0f, 1e-3f);
    EXPECT_NEAR(s, 1.0f, 1e-3f);
    EXPECT_NEAR(v, 1.0f, 1e-3f);
  }
  {
    // Max == R, g < b branch
    const auto [h, s, v] = rgb2hsv(1.0f, 0.0f, 0.5f);
    EXPECT_GT(h, 0.5f);
    EXPECT_NEAR(s, 1.0f, 1e-3f);
    EXPECT_NEAR(v, 1.0f, 1e-3f);
  }
}

TEST(color_maps, rgb2hsv_GrayHasZeroHueAndSaturation)
{
  using namespace mrpt::img;
  const auto [h, s, v] = rgb2hsv(0.4f, 0.4f, 0.4f);
  EXPECT_NEAR(h, 0.0f, 1e-3f);
  EXPECT_NEAR(s, 0.0f, 1e-3f);
  EXPECT_NEAR(v, 0.4f, 1e-3f);
}

TEST(color_maps, rgb2hsv_BlackHasZeroSaturation)
{
  using namespace mrpt::img;
  const auto [h, s, v] = rgb2hsv(0.0f, 0.0f, 0.0f);
  EXPECT_NEAR(h, 0.0f, 1e-3f);
  EXPECT_NEAR(s, 0.0f, 1e-3f);
  EXPECT_NEAR(v, 0.0f, 1e-3f);
}

TEST(color_maps, rgb2hsv_ClampsOutOfRangeInputs)
{
  using namespace mrpt::img;
  const auto [h1, s1, v1] = rgb2hsv(2.0f, -1.0f, 5.0f);
  const auto [h2, s2, v2] = rgb2hsv(1.0f, 0.0f, 1.0f);
  EXPECT_NEAR(h1, h2, 1e-3f);
  EXPECT_NEAR(s1, s2, 1e-3f);
  EXPECT_NEAR(v1, v2, 1e-3f);
}

TEST(color_maps, hsv2rgb_rgb2hsv_RoundTrip)
{
  using namespace mrpt::img;
  for (float h = 0.05f; h < 1.0f; h += 0.1f)
  {
    const auto c = hsv2rgb(h, 0.7f, 0.9f);
    const auto [h2, s2, v2] = rgb2hsv(c.R, c.G, c.B);
    EXPECT_NEAR(h, h2, 1e-2f) << "h=" << h;
    EXPECT_NEAR(s2, 0.7f, 1e-2f) << "h=" << h;
    EXPECT_NEAR(v2, 0.9f, 1e-2f) << "h=" << h;
  }
}
