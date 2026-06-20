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

#include <mrpt/img/color_maps.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <mrpt/math/interp_fit.hpp>

/*-------------------------------------------------------------
          hsv2rgb
-------------------------------------------------------------*/
mrpt::img::TColorf mrpt::img::hsv2rgb(float h, float s, float v)
{
  // See: http://en.wikipedia.org/wiki/HSV_color_space

  h = std::clamp(h, 0.0f, 1.0f);
  s = std::clamp(s, 0.0f, 1.0f);
  v = std::clamp(v, 0.0f, 1.0f);

  const float hf = h * 6.0f;
  const int hi = static_cast<int>(hf) % 6;  // h ∈ [0,1], so truncation == floor
  const float f = hf - static_cast<float>(hi);

  const float p = v * (1.0f - s);
  const float q = v * (1.0f - f * s);
  const float t = v * (1.0f - (1.0f - f) * s);

  mrpt::img::TColorf ret;
  auto& r = ret.R;
  auto& g = ret.G;
  auto& b = ret.B;

  switch (hi)
  {
    default:
      THROW_EXCEPTION("Should not reach here");

    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
      r = v;
      g = p;
      b = q;
      break;
  }

  return ret;
}

/*-------------------------------------------------------------
          rgb2hsv
-------------------------------------------------------------*/
std::tuple<float, float, float> mrpt::img::rgb2hsv(float r, float g, float b)
{
  // See: http://en.wikipedia.org/wiki/HSV_color_space
  r = std::clamp(r, 0.0f, 1.0f);
  g = std::clamp(g, 0.0f, 1.0f);
  b = std::clamp(b, 0.0f, 1.0f);

  const float Max = max3(r, g, b);
  const float Min = min3(r, g, b);

  float h = 0;
  float s = 0;
  float v = 0;

  if (Max == Min)
  {
    h = 0;
  }
  else
  {
    if (Max == r)
    {
      if (g >= b)
      {
        h = (g - b) / (6 * (Max - Min));
      }
      else
      {
        h = 1 - ((g - b) / (6 * (Max - Min)));
      }
    }
    else if (Max == g)
    {
      h = (1 / 3.0f) + ((b - r) / (6 * (Max - Min)));
    }
    else
    {
      h = (2 / 3.0f) + ((r - g) / (6 * (Max - Min)));
    }
  }

  if (Max == 0)
  {
    s = 0;
  }
  else
  {
    s = 1 - (Min / Max);
  }

  v = Max;

  return {h, s, v};
}

/*-------------------------------------------------------------
          colormap
-------------------------------------------------------------*/
mrpt::img::TColorf mrpt::img::colormap(const TColormap& color_map, float color_index)
{
  color_index = std::clamp(color_index, 0.0f, 1.0f);

  mrpt::img::TColorf col;

  switch (color_map)
  {
    case cmJET:
      col = jet2rgb(color_index);
      break;
    case cmGRAYSCALE:
      col.R = col.G = col.B = color_index;
      break;
    case cmHOT:
      col = hot2rgb(color_index);
      break;
    default:
      THROW_EXCEPTION("Invalid color_map");
  };
  return col;
}

/*-------------------------------------------------------------
          jet2rgb
-------------------------------------------------------------*/
mrpt::img::TColorf mrpt::img::jet2rgb(const float color_index)
{
  thread_local bool jet_table_done = false;
  thread_local Eigen::VectorXf jet_r;
  thread_local Eigen::VectorXf jet_g;
  thread_local Eigen::VectorXf jet_b;

  // Initialize tables
  if (!jet_table_done)
  {
    jet_table_done = true;

    // Refer to source code of "jet" in MATLAB:
    constexpr float JET_R[] = {
        0.0f,    0.0f,   0.0f,    0.0f,    0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
        0.0f,    0.0f,   0.0f,    0.0f,    0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
        0.0f,    0.0f,   0.0f,    0.0f,    0.0625f, 0.125f, 0.1875f, 0.250f, 0.3125f, 0.375f,
        0.4375f, 0.5f,   0.5625f, 0.625f,  0.6875f, 0.750f, 0.8125f, 0.875f, 0.9375f, 1.0f,
        1.0f,    1.0f,   1.0f,    1.0f,    1.0f,    1.0f,   1.0f,    1.0f,   1.0f,    1.0f,
        1.0f,    1.0f,   1.0f,    1.0f,    1.0f,    1.0f,   0.9375f, 0.875f, 0.8125f, 0.750f,
        0.6875f, 0.625f, 0.5625f, 0.500000};
    constexpr float JET_G[] = {
        0.0f,    0.0f,   0.0f,    0.0f,    0.0f,    0.0f,   0.0f,    0.0f,   0.0625f, 0.125f,
        0.1875f, 0.250f, 0.3125f, 0.375f,  0.4375f, 0.5f,   0.5625f, 0.625f, 0.6875f, 0.750f,
        0.8125f, 0.875f, 0.9375f, 1.0f,    1.0f,    1.0f,   1.0f,    1.0f,   1.0f,    1.0f,
        1.0f,    1.0f,   1.0f,    1.0f,    1.0f,    1.0f,   1.0f,    1.0f,   1.0f,    1.0f,
        0.9375f, 0.875f, 0.8125f, 0.750f,  0.6875f, 0.625f, 0.5625f, 0.5f,   0.4375f, 0.375f,
        0.3125f, 0.250f, 0.1875f, 0.125f,  0.0625f, 0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
        0.0f,    0.0f,   0.0f,    0.000000};
    constexpr float JET_B[] = {
        0.5625f, 0.625f, 0.6875f, 0.750f,  0.8125f, 0.875f, 0.9375f, 1.0f,   1.0f,    1.0f,
        1.0f,    1.0f,   1.0f,    1.0f,    1.0f,    1.0f,   1.0f,    1.0f,   1.0f,    1.0f,
        1.0f,    1.0f,   1.0f,    1.0f,    0.9375f, 0.875f, 0.8125f, 0.750f, 0.6875f, 0.625f,
        0.5625f, 0.5f,   0.4375f, 0.375f,  0.3125f, 0.250f, 0.1875f, 0.125f, 0.0625f, 0.0f,
        0.0f,    0.0f,   0.0f,    0.0f,    0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
        0.0f,    0.0f,   0.0f,    0.0f,    0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
        0.0f,    0.0f,   0.0f,    0.000000};
    const int N = sizeof(JET_B) / sizeof(JET_B[0]);

    jet_r.resize(N);
    jet_g.resize(N);
    jet_b.resize(N);
    for (int i = 0; i < N; i++)
    {
      jet_r[i] = JET_R[i];
      jet_g[i] = JET_G[i];
      jet_b[i] = JET_B[i];
    }
  }

  // Return interpolate value:
  return {
      math::interpolate(color_index, jet_r, 0.0f, 1.0f),
      math::interpolate(color_index, jet_g, 0.0f, 1.0f),
      math::interpolate(color_index, jet_b, 0.0f, 1.0f)};
}

mrpt::img::TColorf mrpt::img::hot2rgb(const float color_index)
{
  thread_local bool table_done = false;
  thread_local Eigen::VectorXf hot_r;
  thread_local Eigen::VectorXf hot_g;
  thread_local Eigen::VectorXf hot_b;

  // Initialize tables
  if (!table_done)
  {
    table_done = true;

    // Refer to source code of "hot" in MATLAB:
    constexpr float HOT_R[] = {
        0.041667f, 0.0833f,   0.125f,    0.166667f, 0.2083f,   0.250f,    0.291667f, 0.3333f,
        0.375f,    0.416667f, 0.4583f,   0.5f,      0.541667f, 0.5833f,   0.625f,    0.666667f,
        0.7083f,   0.750f,    0.791667f, 0.8333f,   0.875f,    0.916667f, 0.9583f,   1.0f,
        1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,
        1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,
        1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,
        1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,
        1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f};
    constexpr float HOT_G[] = {
        0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,
        0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,
        0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,      0.0f,
        0.041667f, 0.0833f,   0.125f,    0.166667f, 0.2083f,   0.250f,    0.291667f, 0.3333f,
        0.375f,    0.416667f, 0.4583f,   0.5f,      0.541667f, 0.5833f,   0.625f,    0.666667f,
        0.7083f,   0.750f,    0.791667f, 0.8333f,   0.875f,    0.916667f, 0.9583f,   1.0f,
        1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,
        1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f,      1.0f};
    constexpr float HOT_B[] = {0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
                               0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
                               0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
                               0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
                               0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
                               0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,   0.0f,    0.0f,
                               0.0625f, 0.125f, 0.1875f, 0.250f, 0.3125f, 0.375f, 0.4375f, 0.5f,
                               0.5625f, 0.625f, 0.6875f, 0.750f, 0.8125f, 0.875f, 0.9375f, 1.0f};
    const int N = sizeof(HOT_B) / sizeof(HOT_B[0]);

    hot_r.resize(N);
    hot_g.resize(N);
    hot_b.resize(N);
    for (int i = 0; i < N; i++)
    {
      hot_r[i] = HOT_R[i];
      hot_g[i] = HOT_G[i];
      hot_b[i] = HOT_B[i];
    }
  }

  // Return interpolate value:
  return {
      math::interpolate(color_index, hot_r, 0.0f, 1.0f),
      math::interpolate(color_index, hot_g, 0.0f, 1.0f),
      math::interpolate(color_index, hot_b, 0.0f, 1.0f)};
}
