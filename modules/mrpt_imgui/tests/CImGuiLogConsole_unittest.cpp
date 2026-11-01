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
#include <mrpt/imgui/CImGuiLogConsole.h>

using namespace mrpt::imgui;
using mrpt::system::LVL_DEBUG;
using mrpt::system::LVL_ERROR;
using mrpt::system::LVL_INFO;
using mrpt::system::LVL_WARN;

TEST(CImGuiLogConsole, PushSplitsLines)
{
  CImGuiLogConsole c;
  c.push(LVL_INFO, "src", "one\ntwo\r\nthree\n");
  EXPECT_EQ(c.size(), 3u);
  const auto all = c.filtered();
  ASSERT_EQ(all.size(), 3u);
  EXPECT_EQ(all[0].text, "one");
  EXPECT_EQ(all[1].text, "two");
  EXPECT_EQ(all[2].text, "three");
  c.push(LVL_INFO, "src", "");
  EXPECT_EQ(c.size(), 4u);
}

TEST(CImGuiLogConsole, Filters)
{
  CImGuiLogConsole c;
  c.push(LVL_DEBUG, "a", "debug message");
  c.push(LVL_INFO, "a", "Info message");
  c.push(LVL_WARN, "b", "careful");
  c.push(LVL_ERROR, "b", "failure");
  EXPECT_EQ(c.filtered().size(), 4u);

  c.filter.minLevel = LVL_WARN;
  EXPECT_EQ(c.filtered().size(), 2u);

  c.filter.minLevel = LVL_DEBUG;
  c.filter.text = "MESSAGE";  // case insensitive
  EXPECT_EQ(c.filtered().size(), 2u);
  c.filter.text = "b";                 // also matches sources
  EXPECT_EQ(c.filtered().size(), 3u);  // "debug", and both from source b

  c.filter.text.clear();
  c.filter.hiddenSources.insert("b");
  EXPECT_EQ(c.filtered().size(), 2u);

  const auto srcs = c.sources();
  ASSERT_EQ(srcs.size(), 2u);
  EXPECT_EQ(srcs[0], "a");
  EXPECT_EQ(srcs[1], "b");
}

TEST(CImGuiLogConsole, MaxEntriesAndClear)
{
  CImGuiLogConsole c;
  c.maxEntries = 3;
  for (int i = 0; i < 10; i++)
  {
    c.push(LVL_INFO, "", std::to_string(i));
  }
  const auto all = c.filtered();
  ASSERT_EQ(all.size(), 3u);
  EXPECT_EQ(all.front().text, "7");
  const auto v = c.version();
  c.clear();
  EXPECT_EQ(c.size(), 0u);
  EXPECT_GT(c.version(), v);
}

TEST(CImGuiLogConsole, FormatAndText)
{
  CImGuiLogConsole c;
  c.showTimestamps = false;
  c.push(LVL_WARN, "nav", "slow");
  EXPECT_EQ(c.filteredAsText(), "[WARN] [nav] slow\n");
  c.showSources = false;
  EXPECT_EQ(c.filteredAsText(), "[WARN] slow\n");
}

TEST(CImGuiLogConsole, AttachToLogger)
{
  mrpt::system::COutputLogger logger("myLogger");
  logger.logging_enable_console_output = false;
  {
    CImGuiLogConsole c;
    c.attach(logger);
    logger.logStr(LVL_INFO, "hello");
    logger.logStr(LVL_ERROR, "bad\n");
    const auto all = c.filtered();
    ASSERT_EQ(all.size(), 2u);
    EXPECT_EQ(all[0].text, "hello");
    EXPECT_EQ(all[0].source, "myLogger");
    EXPECT_EQ(all[1].level, LVL_ERROR);
    EXPECT_EQ(all[1].text, "bad");
  }
  // The console detached on destruction: logging now must not crash.
  logger.logStr(LVL_INFO, "after");

  CImGuiLogConsole c2;
  c2.attach(logger);
  c2.detach(logger);
  logger.logStr(LVL_INFO, "ignored");
  EXPECT_EQ(c2.size(), 0u);
}
