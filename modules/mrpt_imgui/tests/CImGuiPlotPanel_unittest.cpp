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
#include <mrpt/imgui/CImGuiPlotPanel.h>

#include <algorithm>
#include <cmath>

using namespace mrpt::imgui;

TEST(CImGuiPlotPanel, SeriesManagement)
{
  CImGuiPlotPanel p;
  p.addPoint("a", 0, 1);
  p.addPoint("a", 1, 2);
  p.setSeries("b", {0, 1, 2}, {5, 6});  // truncated to the shorter
  ASSERT_NE(p.series("a"), nullptr);
  EXPECT_EQ(p.series("a")->x.size(), 2u);
  EXPECT_EQ(p.series("b")->x.size(), 2u);
  EXPECT_EQ(p.series("c"), nullptr);
  EXPECT_EQ(p.allSeries().size(), 2u);
  EXPECT_EQ(p.allSeries()[0].name, "a");  // insertion order kept

  p.setBand("b", {}, {4, 5}, {6, 7});
  EXPECT_EQ(p.series("b")->bandLow.size(), 2u);

  p.clearData();
  EXPECT_TRUE(p.series("a")->x.empty());
  p.removeSeries("a");
  EXPECT_EQ(p.series("a"), nullptr);
  p.clear();
  EXPECT_TRUE(p.allSeries().empty());
}

TEST(CImGuiPlotPanel, BandWithXCreatesNaNSeries)
{
  CImGuiPlotPanel p;
  p.setBand("conf", {0, 1, 2}, {-1, -1, -1}, {1, 1, 1});
  const auto* s = p.series("conf");
  ASSERT_NE(s, nullptr);
  ASSERT_EQ(s->y.size(), 3u);
  EXPECT_TRUE(std::isnan(s->y[0]));
}

TEST(CImGuiPlotPanel, MaxPointsTrimming)
{
  CImGuiPlotPanel p;
  p.maxPoints = 100;
  for (int i = 0; i < 5000; i++)
  {
    p.addPoint("s", i, i);
  }
  const auto* s = p.series("s");
  // Trimming is amortized: never more than a chunk over the limit.
  EXPECT_LE(s->x.size(), 100u + 1024u);
  EXPECT_EQ(s->x.back(), 4999.0);
  EXPECT_EQ(s->x.size(), s->y.size());
}

TEST(CImGuiPlotPanel, RollingSpanTrimming)
{
  CImGuiPlotPanel p;
  p.rollingSpan = 10;
  for (int i = 0; i < 20000; i++)
  {
    p.addPoint("s", i * 0.01, 0);  // 200 s of data
  }
  const auto* s = p.series("s");
  EXPECT_GE(s->x.back() - s->x.front(), 20.0 - 1e-6);  // keeps 2x the span
  EXPECT_LT(s->x.back() - s->x.front(), 20.0 + 1024 * 0.01 + 1e-6);
}

TEST(CImGuiPlotPanel, ToCSV)
{
  CImGuiPlotPanel p;
  p.setSeries("a", {0, 1}, {10, 11});
  p.setSeries("b", {0}, {20});
  EXPECT_EQ(p.toCSV(), "a_x,a_y,b_x,b_y\n0,10,0,20\n1,11,,\n");
}

TEST(CImGuiPlotPanel, DecimateMinMaxKeepsExtremes)
{
  std::vector<double> x;
  std::vector<double> y;
  for (int i = 0; i < 100000; i++)
  {
    x.push_back(i);
    y.push_back(std::sin(i * 0.01));
  }
  y[54321] = 100;   // spike
  y[12345] = -100;  // dip
  const auto idx = CImGuiPlotPanel::decimateMinMax(x, y, 0, 99999, 500);
  EXPECT_LE(idx.size(), 2u * 500u + 2u);
  EXPECT_GE(idx.size(), 500u);
  EXPECT_TRUE(std::is_sorted(idx.begin(), idx.end()));
  EXPECT_NE(std::find(idx.begin(), idx.end(), 54321u), idx.end());
  EXPECT_NE(std::find(idx.begin(), idx.end(), 12345u), idx.end());
}

TEST(CImGuiPlotPanel, DecimateMinMaxRangeAndSmallInputs)
{
  std::vector<double> x;
  std::vector<double> y;
  for (int i = 0; i < 1000; i++)
  {
    x.push_back(i);
    y.push_back(i);
  }
  // Few points in range: all of them, plus one neighbor at each side.
  const auto idx = CImGuiPlotPanel::decimateMinMax(x, y, 100, 110, 50);
  ASSERT_EQ(idx.size(), 13u);
  EXPECT_EQ(idx.front(), 99u);
  EXPECT_EQ(idx.back(), 111u);

  EXPECT_TRUE(CImGuiPlotPanel::decimateMinMax({}, {}, 0, 1, 10).empty());
}
