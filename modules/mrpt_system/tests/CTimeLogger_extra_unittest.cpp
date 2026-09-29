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
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>

#include <fstream>
#include <map>
#include <sstream>
#include <string>

using namespace mrpt::system;

namespace
{
std::string readFile(const std::string& fil)
{
  std::ifstream f(fil);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}
}  // namespace

TEST(CTimeLogger, UserMeasuresStatistics)
{
  CTimeLogger tl(true, "user_measures");
  tl.logging_enable_console_output = false;
  EXPECT_EQ(tl.getName(), "user_measures");

  for (const double v : {1.0, 2.0, 3.0})
  {
    tl.registerUserMeasure("my.value", v);
  }
  tl.registerUserMeasure("my.time", 0.5, true /*is a time*/);

  std::map<std::string, CTimeLogger::TCallStats> stats;
  tl.getStats(stats);
  ASSERT_EQ(stats.count("my.value"), 1u);
  const auto& s = stats.at("my.value");
  EXPECT_EQ(s.n_calls, 3u);
  EXPECT_DOUBLE_EQ(s.min_t, 1.0);
  EXPECT_DOUBLE_EQ(s.max_t, 3.0);
  EXPECT_DOUBLE_EQ(s.mean_t, 2.0);
  EXPECT_DOUBLE_EQ(s.total_t, 6.0);
  EXPECT_DOUBLE_EQ(s.last_t, 3.0);

  // The text report has a line per section, with a tree-like layout for
  // names with dots:
  const auto text = tl.getStatsAsText();
  EXPECT_NE(text.find("my"), std::string::npos);
  EXPECT_NE(text.find("value"), std::string::npos);
  EXPECT_NE(text.find("user_measures"), std::string::npos);

  // Nothing is recorded when the logger is disabled:
  tl.disable();
  EXPECT_FALSE(tl.isEnabled());
  tl.registerUserMeasure("my.value", 100.0);
  tl.getStats(stats);
  EXPECT_EQ(stats.at("my.value").n_calls, 3u);
  tl.enable();
  EXPECT_TRUE(tl.isEnabled());

  EXPECT_NO_THROW(tl.dumpAllStats(100));
}

TEST(CTimeLogger, WholeHistoryAndFileOutputs)
{
  CTimeLogger tl(true, "history_logger", true /*keep the whole history*/);
  tl.logging_enable_console_output = false;
  EXPECT_TRUE(tl.isEnabledKeepWholeHistory());

  for (const double v : {10.0, 20.0, 60.0})
  {
    tl.registerUserMeasure("a-section+with*chars.and.dots", v);
  }
  tl.registerUserMeasure("second", 5.0, true);

  const auto csv = getTempFileName() + ".csv";
  tl.saveToCSVFile(csv);
  const auto csvText = readFile(csv);
  EXPECT_NE(csvText.find("FUNCTION"), std::string::npos);
  EXPECT_NE(csvText.find("a-section+with*chars.and.dots"), std::string::npos);
  // The history is appended to the row of the section:
  EXPECT_NE(csvText.find("10.000000, 20.000000, 60.000000"), std::string::npos);
  deleteFile(csv);

  const auto mfile = getTempFileName() + "_stats.m";
  tl.saveToMFile(mfile);
  const auto m = readFile(mfile);
  EXPECT_NE(m.find("function [s] = "), std::string::npos);
  EXPECT_NE(m.find("s.names={"), std::string::npos);
  // Counts, and the true mean (30) and not the total (90):
  EXPECT_NE(m.find("s.count=["), std::string::npos);
  EXPECT_NE(m.find("3,"), std::string::npos);
  EXPECT_NE(m.find("3.000000e+01,"), std::string::npos) << m;
  EXPECT_EQ(m.find("9.000000e+01"), std::string::npos) << "the total was written as the mean";
  // The history goes into a valid Matlab identifier:
  EXPECT_NE(m.find("s.whole.a_section_with_chars_and_dots=["), std::string::npos) << m;
  deleteFile(mfile);

  // Stats are cleared, but sections stay usable:
  tl.clear();
  std::map<std::string, CTimeLogger::TCallStats> stats;
  tl.getStats(stats);
  for (const auto& [name, st] : stats)
  {
    EXPECT_EQ(st.n_calls, 0u) << name;
  }
  tl.enableKeepWholeHistory(false);
  EXPECT_FALSE(tl.isEnabledKeepWholeHistory());
}

TEST(CTimeLogger, SaveAtDestructionAndScopedEntries)
{
  const std::string name = "scoped_logger";
  const std::string mfile = fileNameStripInvalidChars(name + ".m");
  {
    CTimeLogger tl(true, name);
    tl.logging_enable_console_output = false;
    CTimeLoggerSaveAtDtor saver(tl);
    {
      CTimeLoggerEntry e(tl, "scoped.section");
    }
    {
      CTimeLoggerEntry e(tl, "unscoped.section");
      e.stop();
      e.stop();  // harmless to stop twice
    }
    tl.registerUserMeasure("manual", 1.0);
  }
  ASSERT_TRUE(fileExists(mfile));
  const auto m = readFile(mfile);
  EXPECT_NE(m.find("scoped.section"), std::string::npos);
  EXPECT_NE(m.find("manual"), std::string::npos);
  deleteFile(mfile);
}

TEST(CTimeLogger, EnterLeaveOfUnknownSectionsAndCopies)
{
  CTimeLogger tl;
  tl.logging_enable_console_output = false;
  // Leaving something that was never entered does nothing bad:
  EXPECT_EQ(tl.leave("never-entered"), 0.0);

  tl.enter("section");
  const double dt = tl.leave("section");
  EXPECT_GE(dt, 0.0);

  // Copy and move keep the enabled state:
  CTimeLogger copy(tl);
  EXPECT_EQ(copy.isEnabled(), tl.isEnabled());
  tl.disable();
  CTimeLogger other;
  other = tl;
  EXPECT_FALSE(other.isEnabled());
  CTimeLogger moved(std::move(other));
  EXPECT_FALSE(moved.isEnabled());
  CTimeLogger moved2;
  moved2 = std::move(copy);
  EXPECT_TRUE(moved2.isEnabled());
}
