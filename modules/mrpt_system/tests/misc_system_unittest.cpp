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
#include <mrpt/system/CRateTimer.h>
#include <mrpt/system/crc.h>
#include <mrpt/system/hyperlink.h>
#include <mrpt/system/memory.h>
#include <mrpt/system/progress.h>
#include <mrpt/system/scheduler.h>

#include <cstdint>
#include <string>
#include <vector>

// ------------------------------- progress ----------------------------------

TEST(progress, emptyAndFullBars)
{
  // An empty bar is all blanks, a full one all blocks; either way the visible
  // length is the requested one plus the two brackets.
  const std::string empty = mrpt::system::progress(0.0, 10);
  const std::string full = mrpt::system::progress(1.0, 10);

  EXPECT_EQ(empty.front(), '[');
  EXPECT_EQ(empty.back(), ']');
  EXPECT_NE(empty, full);

  EXPECT_NE(full.find("█"), std::string::npos);  // full block
  EXPECT_EQ(empty.find("█"), std::string::npos);
}

TEST(progress, partialBarUsesAnIntermediatePhase)
{
  // A ratio that does not land on a whole character exercises the "phase"
  // branch, which appends a partially-filled block:
  const std::string s = mrpt::system::progress(0.55, 10);
  EXPECT_NE(s.find("█"), std::string::npos);
  EXPECT_NE(s, mrpt::system::progress(0.5, 10));
}

TEST(progress, withoutBrackets)
{
  const std::string s = mrpt::system::progress(0.5, 10, false);
  EXPECT_NE(s.front(), '[');
  EXPECT_NE(s.back(), ']');
}

TEST(progress, invalidArgumentsAreRejected)
{
  EXPECT_ANY_THROW(mrpt::system::progress(-0.1, 10));
  EXPECT_ANY_THROW(mrpt::system::progress(1.1, 10));
  EXPECT_ANY_THROW(mrpt::system::progress(0.5, 0));
}

// ------------------------------- hyperlink ---------------------------------

TEST(hyperlink, forcedEscapeSequenceFormat)
{
  const std::string s = mrpt::system::hyperlink("MRPT", "https://www.mrpt.org/", true);
  EXPECT_NE(s.find("https://www.mrpt.org/"), std::string::npos);
  EXPECT_NE(s.find("MRPT"), std::string::npos);
  EXPECT_NE(s.find("\033]8;;"), std::string::npos);
}

TEST(hyperlink, plainTextFallback)
{
  // Under a test runner stdout is not a terminal, so without force_use_format
  // the plain-text forms are used:
  EXPECT_EQ(mrpt::system::hyperlink("MRPT", "https://www.mrpt.org/", false, false), "MRPT");
  EXPECT_EQ(
      mrpt::system::hyperlink("MRPT", "https://www.mrpt.org/", false, true),
      "MRPT (https://www.mrpt.org/)");
}

// --------------------------------- crc -------------------------------------

TEST(crc, crc16)
{
  const std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04};

  const uint16_t a = mrpt::system::compute_CRC16(data);
  const uint16_t b = mrpt::system::compute_CRC16(data.data(), data.size());
  EXPECT_EQ(a, b);

  // A different message must (very likely) give a different checksum:
  const std::vector<uint8_t> other = {0x01, 0x02, 0x03, 0x05};
  EXPECT_NE(mrpt::system::compute_CRC16(other), a);

  // The std::vector overloads reject an empty message (they index &data[0]),
  // while the pointer ones simply return the seed:
  EXPECT_ANY_THROW((void)mrpt::system::compute_CRC16(std::vector<uint8_t>{}));
  EXPECT_EQ(mrpt::system::compute_CRC16(data.data(), 0), 0U);
}

TEST(crc, crc32VectorAndPointerOverloadsAgree)
{
  const std::vector<uint8_t> data = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};

  const uint32_t a = mrpt::system::compute_CRC32(data);
  const uint32_t b = mrpt::system::compute_CRC32(data.data(), data.size());
  EXPECT_EQ(a, b);

  const std::vector<uint8_t> other = {0xAA, 0xBB, 0xCC, 0xDD, 0xEF};
  EXPECT_NE(mrpt::system::compute_CRC32(other), a);

  EXPECT_ANY_THROW((void)mrpt::system::compute_CRC32(std::vector<uint8_t>{}));
  EXPECT_NO_THROW((void)mrpt::system::compute_CRC32(data.data(), 0));
}

// ------------------------------- scheduler ---------------------------------

TEST(scheduler, changeCurrentThreadPriority)
{
  // Raising the priority requires privileges we may not have; the function
  // must warn and carry on, never throw.
  for (const auto p :
       {mrpt::system::tpLowests, mrpt::system::tpLower, mrpt::system::tpLow, mrpt::system::tpNormal,
        mrpt::system::tpHigh, mrpt::system::tpHigher, mrpt::system::tpHighest})
  {
    EXPECT_NO_THROW(mrpt::system::changeCurrentThreadPriority(p));
  }
}

TEST(scheduler, changeCurrentProcessPriority)
{
  for (const auto p :
       {mrpt::system::ppIdle, mrpt::system::ppNormal, mrpt::system::ppHigh,
        mrpt::system::ppVeryHigh})
  {
    EXPECT_NO_THROW(mrpt::system::changeCurrentProcessPriority(p));
  }
}

// -------------------------------- memory -----------------------------------

TEST(memory, getMemoryUsage)
{
  // Not every platform can report it, so only require that it does not throw:
  EXPECT_NO_THROW((void)mrpt::system::getMemoryUsage());
}

// ------------------------------ CRateTimer ---------------------------------

TEST(CRateTimer, rateAccessors)
{
  mrpt::system::CRateTimer t(10.0);
  EXPECT_EQ(t.rate(), 10.0);

  t.setRate(50.0);
  EXPECT_EQ(t.rate(), 50.0);

  EXPECT_ANY_THROW(t.setRate(0.0));
  EXPECT_ANY_THROW(t.setRate(-1.0));
}

TEST(CRateTimer, sleepReturnsWhetherItHadToWait)
{
  mrpt::system::CRateTimer t(1000.0);  // 1 ms period

  // The first call has no previous tick to wait for:
  t.sleep();
  // ...and a second, immediate one must have had to wait:
  EXPECT_TRUE(t.sleep());
}
