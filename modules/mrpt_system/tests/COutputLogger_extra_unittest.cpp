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
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/filesystem.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace mrpt::system;

namespace
{
/** Makes a logger not print anything to the console, and keep a record. (It is
 * configured in place: copies of a logger do not keep its configuration.) */
void makeQuiet(COutputLogger& l)
{
  l.logging_enable_console_output = false;
  l.logging_enable_keep_record = true;
  l.setMinLoggingLevel(LVL_DEBUG);
}

std::vector<std::string> g_callbackMessages;
void freeFunctionCallback(
    std::string_view msg, VerbosityLevel, std::string_view, mrpt::Clock::time_point)
{
  g_callbackMessages.emplace_back(msg);
}
}  // namespace

TEST(COutputLogger, HistoryAndLastMessage)
{
  COutputLogger l("test_history");
  makeQuiet(l);
  EXPECT_EQ(l.getLoggerName(), "test_history");
  // Nothing logged yet: no last message, and asking for it must be harmless
  EXPECT_TRUE(l.getLoggerLastMsg().empty());

  l.logStr(LVL_INFO, "first message");
  l.logStr(LVL_WARN, "second message\n");
  const std::string all = l.getLogAsString();
  EXPECT_NE(all.find("first message"), std::string::npos);
  EXPECT_NE(all.find("second message"), std::string::npos);
  EXPECT_NE(all.find("test_history"), std::string::npos);
  EXPECT_NE(all.find("WARN"), std::string::npos);

  const auto last = l.getLoggerLastMsg();
  EXPECT_NE(last.find("second message"), std::string::npos);
  std::string last2;
  l.getLoggerLastMsg(last2);
  EXPECT_EQ(last, last2);

  // Reset drops everything, and restores the defaults:
  l.setLoggerName("changed");
  l.setMinLoggingLevel(LVL_ERROR);
  l.loggerReset();
  EXPECT_TRUE(l.getLogAsString().empty());
  EXPECT_TRUE(l.getLoggerLastMsg().empty());
  EXPECT_EQ(l.getLoggerName(), "COutputLogger");
  EXPECT_TRUE(l.logging_enable_console_output);
  EXPECT_FALSE(l.logging_enable_keep_record);
  EXPECT_TRUE(l.isLoggingLevelVisible(LVL_INFO));
}

TEST(COutputLogger, FormattedAndConditionalMessages)
{
  COutputLogger l("test_fmt");
  makeQuiet(l);
  l.logFmt(LVL_INFO, "value=%d text=%s", 42, "abc");
  EXPECT_NE(l.getLoggerLastMsg().find("value=42 text=abc"), std::string::npos);

  // Longer than any small fixed buffer:
  const std::string longStr(5000, 'x');
  l.logFmt(LVL_INFO, "%s", longStr.c_str());
  EXPECT_NE(l.getLoggerLastMsg().find(longStr), std::string::npos);

  // A null format is ignored:
  const auto before = l.getLogAsString();
  l.logFmt(LVL_INFO, nullptr);
  EXPECT_EQ(l.getLogAsString(), before);

  l.logCond(LVL_INFO, false, "must not appear");
  EXPECT_EQ(l.getLogAsString(), before);
  l.logCond(LVL_INFO, true, "must appear");
  EXPECT_NE(l.getLoggerLastMsg().find("must appear"), std::string::npos);
}

TEST(COutputLogger, VerbosityLevels)
{
  COutputLogger l("test_levels");
  l.logging_enable_console_output = false;
  l.setVerbosityLevel(LVL_WARN);
  EXPECT_FALSE(l.isLoggingLevelVisible(LVL_DEBUG));
  EXPECT_FALSE(l.isLoggingLevelVisible(LVL_INFO));
  EXPECT_TRUE(l.isLoggingLevelVisible(LVL_WARN));
  EXPECT_TRUE(l.isLoggingLevelVisible(LVL_ERROR));

  l.setMinLoggingLevel(LVL_DEBUG);
  EXPECT_TRUE(l.isLoggingLevelVisible(LVL_DEBUG));
}

TEST(COutputLogger, Callbacks)
{
  COutputLogger l("test_cb");
  makeQuiet(l);

  // A capturing callback, with its own minimum level:
  std::vector<std::pair<std::string, VerbosityLevel>> received;
  l.logRegisterCallback(
      [&received](
          std::string_view msg, const VerbosityLevel level, std::string_view name,
          mrpt::Clock::time_point)
      {
        EXPECT_EQ(name, "test_cb");
        received.emplace_back(std::string(msg), level);
      });
  l.setVerbosityLevelForCallbacks(LVL_WARN);
  l.logStr(LVL_INFO, "ignored by the callback");
  l.logStr(LVL_ERROR, "seen by the callback");
  ASSERT_EQ(received.size(), 1u);
  EXPECT_EQ(received[0].first, "seen by the callback");
  EXPECT_EQ(received[0].second, LVL_ERROR);

  // A free function callback can be deregistered by identity:
  g_callbackMessages.clear();
  l.setVerbosityLevelForCallbacks(LVL_DEBUG);
  l.logRegisterCallback(&freeFunctionCallback);
  l.logStr(LVL_INFO, "to the function");
  ASSERT_EQ(g_callbackMessages.size(), 1u);
  EXPECT_TRUE(l.logDeregisterCallback(&freeFunctionCallback));
  l.logStr(LVL_INFO, "after removing it");
  EXPECT_EQ(g_callbackMessages.size(), 1u);
  // ...but only once:
  EXPECT_FALSE(l.logDeregisterCallback(&freeFunctionCallback));
}

TEST(COutputLogger, WriteLogToFileAndDumpToConsole)
{
  // (a name of our own, so the default log file cannot be someone else's)
  const std::string loggerName =
      "test/write*log_" + mrpt::system::extractFileName(mrpt::system::getTempFileName());
  COutputLogger l(loggerName);
  makeQuiet(l);
  l.logStr(LVL_INFO, "line to save");

  const std::string file = mrpt::system::getTempFileName() + "_logger.log";
  l.writeLogToFile(file);
  std::ifstream f(file);
  ASSERT_TRUE(f.is_open());
  std::stringstream ss;
  ss << f.rdbuf();
  EXPECT_NE(ss.str().find("line to save"), std::string::npos);
  f.close();
  mrpt::system::deleteFile(file);

  // Without a file name, one is made from the logger name without the
  // characters that are not valid in file names:
  const std::string defaultName = mrpt::system::fileNameStripInvalidChars(loggerName) + ".log";
  l.writeLogToFile();
  EXPECT_TRUE(mrpt::system::fileExists(defaultName));
  mrpt::system::deleteFile(defaultName);

  // Cannot write into a non-existent directory:
  EXPECT_ANY_THROW(l.writeLogToFile("/nonexistent-dir-for-test/file.log"));

  // Dumping the record to the console must work too:
  EXPECT_NO_THROW(l.dumpLogToConsole());
}

namespace
{
/** A class that logs through the MRPT_LOG_*() macros, which act on `this`. */
class MyLoggingClass : public COutputLogger
{
 public:
  MyLoggingClass() : COutputLogger("MyLoggingClass")
  {
    logging_enable_console_output = false;
    logging_enable_keep_record = true;
    setMinLoggingLevel(LVL_DEBUG);
  }

  void logAll()
  {
    MRPT_LOG_DEBUG("plain debug");
    MRPT_LOG_INFO("plain info");
    MRPT_LOG_WARN("plain warn");
    MRPT_LOG_ERROR("plain error");
    MRPT_LOG_DEBUG_FMT("fmt debug %d", 1);
    MRPT_LOG_INFO_FMT("fmt info %d", 2);
    MRPT_LOG_WARN_FMT("fmt warn %d", 3);
    MRPT_LOG_ERROR_FMT("fmt error %d", 4);
    MRPT_LOG_DEBUG_STREAM("stream debug " << 5);
    MRPT_LOG_INFO_STREAM("stream info " << 6);
    MRPT_LOG_WARN_STREAM("stream warn " << 7);
    MRPT_LOG_ERROR_STREAM("stream error " << 8);
  }

  void logOnceAndThrottled()
  {
    for (int i = 0; i < 5; i++)
    {
      MRPT_LOG_ONCE_INFO("logged once");
      MRPT_LOG_THROTTLE_WARN(100.0, "throttled warn");
      MRPT_LOG_THROTTLE_ERROR_STREAM(100.0, "throttled error " << i);
      MRPT_LOG_THROTTLE_INFO_FMT(100.0, "throttled fmt %d", i);
    }
  }
};

size_t countOccurrences(const std::string& s, const std::string& sub)
{
  size_t n = 0;
  for (size_t pos = s.find(sub); pos != std::string::npos; pos = s.find(sub, pos + 1))
  {
    n++;
  }
  return n;
}
}  // namespace

TEST(COutputLogger, LoggingMacros)
{
  MyLoggingClass c;
  c.logAll();
  const auto log = c.getLogAsString();
  for (const char* expected :
       {"plain debug", "plain info", "plain warn", "plain error", "fmt debug 1", "fmt info 2",
        "fmt warn 3", "fmt error 4", "stream debug 5", "stream info 6", "stream warn 7",
        "stream error 8"})
  {
    EXPECT_NE(log.find(expected), std::string::npos) << expected;
  }
}

TEST(COutputLogger, OnceAndThrottledMacrosDoNotRepeat)
{
  MyLoggingClass c;
  c.logOnceAndThrottled();
  const auto log = c.getLogAsString();
  EXPECT_EQ(countOccurrences(log, "logged once"), 1u);
  EXPECT_EQ(countOccurrences(log, "throttled warn"), 1u);
  EXPECT_EQ(countOccurrences(log, "throttled error"), 1u);
  EXPECT_EQ(countOccurrences(log, "throttled fmt"), 1u);
}
