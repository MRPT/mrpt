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
#include <mrpt/imgui/CImGuiTimeline.h>

using namespace mrpt::imgui;

TEST(CImGuiTimeline, RangeAndClamping)
{
  CImGuiTimeline tl;
  tl.setRange(10, 0);  // swapped
  EXPECT_DOUBLE_EQ(tl.minTime(), 0);
  EXPECT_DOUBLE_EQ(tl.maxTime(), 10);
  tl.setTime(20);
  EXPECT_DOUBLE_EQ(tl.time(), 10);
  tl.setTime(-5);
  EXPECT_DOUBLE_EQ(tl.time(), 0);
  tl.setTime(4);
  tl.setRange(5, 8);
  EXPECT_DOUBLE_EQ(tl.time(), 5);
}

TEST(CImGuiTimeline, KeyTimesAndStepping)
{
  CImGuiTimeline tl;
  tl.setKeyTimes({3.0, 1.0, 2.0, 5.0});  // unsorted on input
  EXPECT_DOUBLE_EQ(tl.minTime(), 1.0);
  EXPECT_DOUBLE_EQ(tl.maxTime(), 5.0);
  EXPECT_EQ(*tl.currentKeyIndex(), 0u);

  tl.step(1);
  EXPECT_DOUBLE_EQ(tl.time(), 2.0);
  tl.step(2);
  EXPECT_DOUBLE_EQ(tl.time(), 5.0);
  tl.step(1);  // clamped
  EXPECT_DOUBLE_EQ(tl.time(), 5.0);
  EXPECT_EQ(*tl.currentKeyIndex(), 3u);

  // Between keys: forward goes to the next one, backward to the current one.
  tl.setTime(2.5);
  EXPECT_EQ(*tl.currentKeyIndex(), 1u);
  tl.step(1);
  EXPECT_DOUBLE_EQ(tl.time(), 3.0);
  tl.setTime(2.5);
  tl.step(-1);
  EXPECT_DOUBLE_EQ(tl.time(), 2.0);
  tl.step(-10);
  EXPECT_DOUBLE_EQ(tl.time(), 1.0);

  tl.seekToKey(100);
  EXPECT_DOUBLE_EQ(tl.time(), 5.0);
}

TEST(CImGuiTimeline, SteppingWithoutKeys)
{
  CImGuiTimeline tl;
  tl.setRange(0, 1);
  tl.stepSize = 0.25;
  tl.step(2);
  EXPECT_DOUBLE_EQ(tl.time(), 0.5);
  EXPECT_FALSE(tl.currentKeyIndex().has_value());
}

TEST(CImGuiTimeline, PlaybackAndLoop)
{
  CImGuiTimeline tl;
  tl.setRange(0, 10);
  EXPECT_FALSE(tl.update(1.0));  // not playing

  tl.speed = 2.0;
  tl.play();
  EXPECT_TRUE(tl.isPlaying());
  EXPECT_TRUE(tl.update(1.0));
  EXPECT_DOUBLE_EQ(tl.time(), 2.0);

  tl.update(100.0);  // reaches the end and stops
  EXPECT_DOUBLE_EQ(tl.time(), 10.0);
  EXPECT_FALSE(tl.isPlaying());

  tl.play();  // at the end: restarts from the beginning
  EXPECT_DOUBLE_EQ(tl.time(), 0.0);
  tl.loop = true;
  tl.speed = 1.0;
  tl.update(12.0);
  EXPECT_NEAR(tl.time(), 2.0, 1e-9);
  EXPECT_TRUE(tl.isPlaying());

  tl.step(1);  // stepping pauses
  EXPECT_FALSE(tl.isPlaying());
}

TEST(CImGuiTimeline, OnTimeChangedCallback)
{
  CImGuiTimeline tl;
  tl.setRange(0, 10);
  int calls = 0;
  tl.onTimeChanged = [&](double) { calls++; };
  tl.setTime(1);
  tl.setTime(1);  // no change
  tl.jumpToEnd();
  tl.jumpToStart();
  EXPECT_EQ(calls, 3);
}

TEST(CImGuiTimeline, TickColumns)
{
  // Dense ticks collapse to one per pixel column:
  std::vector<double> times;
  for (int i = 0; i <= 1000; i++)
  {
    times.push_back(i * 0.01);
  }
  const auto cols = CImGuiTimeline::tickColumns(times, 0, 10, 100);
  EXPECT_EQ(cols.size(), 100u);
  EXPECT_EQ(cols.front(), 0);
  EXPECT_EQ(cols.back(), 99);

  // Out of range times are ignored:
  const auto c2 = CImGuiTimeline::tickColumns({-1, 5, 20}, 0, 10, 11);
  ASSERT_EQ(c2.size(), 1u);
  EXPECT_EQ(c2[0], 5);

  EXPECT_TRUE(CImGuiTimeline::tickColumns({1, 2}, 0, 0, 10).empty());
  EXPECT_TRUE(CImGuiTimeline::tickColumns({1, 2}, 0, 10, 0).empty());
}

TEST(CImGuiTimeline, TimeLabel)
{
  CImGuiTimeline tl;
  tl.setRange(100, 200);
  tl.setTime(112.5);
  EXPECT_EQ(tl.timeLabel(), "t = 12.500 s");
  tl.timeFormatter = [](double t) { return "#" + std::to_string(static_cast<int>(t)); };
  EXPECT_EQ(tl.timeLabel(), "#112");
}

TEST(CImGuiTimeline, KeysWithTheSameTime)
{
  // E.g. an action and a sensory frame with the same timestamp:
  CImGuiTimeline tl;
  tl.setKeyTimes({0.0, 1.0, 1.0, 1.0, 2.0});
  int calls = 0;
  tl.onTimeChanged = [&calls](double) { calls++; };

  tl.seekToKey(1);
  EXPECT_EQ(*tl.currentKeyIndex(), 1u);
  tl.seekToKey(2);  // same time, another key: still notified
  EXPECT_EQ(*tl.currentKeyIndex(), 2u);
  EXPECT_EQ(calls, 2);

  // Stepping visits every key:
  tl.seekToKey(0);
  for (size_t expected = 1; expected < 5; expected++)
  {
    tl.step(1);
    EXPECT_EQ(*tl.currentKeyIndex(), expected);
  }
  for (size_t expected = 3;; expected--)
  {
    tl.step(-1);
    EXPECT_EQ(*tl.currentKeyIndex(), expected);
    if (expected == 0)
    {
      break;
    }
  }

  // Setting the time directly picks the last key of that time:
  tl.setTime(1.0);
  EXPECT_EQ(*tl.currentKeyIndex(), 3u);
}
