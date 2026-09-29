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
#include <mrpt/io/CPipe.h>

#include <chrono>
#include <string>
#include <thread>

using namespace mrpt::io;
using namespace std::chrono_literals;

namespace
{
struct PipePair
{
  std::unique_ptr<CPipeReadEndPoint> rd;
  std::unique_ptr<CPipeWriteEndPoint> wr;
  PipePair() { CPipe::createPipe(rd, wr); }
};
}  // namespace

#if !defined(_WIN32)
TEST(CPipe, ReadWithTimeoutReturnsWhenNothingArrives)
{
  PipePair p;
  p.rd->timeout_read_start_us = 100000;  // 100 ms
  p.rd->timeout_read_between_us = 50000;

  char buf[8] = {};
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(p.rd->Read(buf, sizeof(buf)), 0u);
  const auto dt = std::chrono::steady_clock::now() - t0;
  EXPECT_GE(dt, 80ms);
  EXPECT_LT(dt, 3000ms);
  // A timeout is not an error: the pipe stays open
  EXPECT_TRUE(p.rd->isOpen());
}

TEST(CPipe, ReadWithTimeoutReturnsWhatArrivedInTime)
{
  PipePair p;
  p.rd->timeout_read_start_us = 1000000;
  p.rd->timeout_read_between_us = 100000;

  // The writer sends 3 bytes, and the 3 remaining ones much later:
  std::thread writer(
      [&p]()
      {
        std::this_thread::sleep_for(20ms);
        p.wr->Write("abc", 3);
        std::this_thread::sleep_for(600ms);
        p.wr->Write("def", 3);
      });

  char buf[8] = {};
  const size_t n = p.rd->Read(buf, 6);
  EXPECT_EQ(n, 3u) << "gave up waiting between chunks";
  EXPECT_EQ(std::string(buf, n), "abc");
  writer.join();

  // The late data is there for the next read:
  p.rd->timeout_read_start_us = 100000;
  EXPECT_EQ(p.rd->Read(buf, 3), 3u);
  EXPECT_EQ(std::string(buf, 3), "def");
}

TEST(CPipe, ReadWithTimeoutCollectsAllOfASlowlySentBlock)
{
  PipePair p;
  p.rd->timeout_read_start_us = 1000000;
  p.rd->timeout_read_between_us = 1000000;
  std::thread writer(
      [&p]()
      {
        for (const char* chunk : {"12", "34", "56"})
        {
          std::this_thread::sleep_for(30ms);
          p.wr->Write(chunk, 2);
        }
      });
  char buf[8] = {};
  EXPECT_EQ(p.rd->Read(buf, 6), 6u);
  EXPECT_EQ(std::string(buf, 6), "123456");
  writer.join();
}

TEST(CPipe, ReadWithTimeoutDetectsTheWriterHangingUp)
{
  PipePair p;
  p.rd->timeout_read_start_us = 2000000;
  p.rd->timeout_read_between_us = 2000000;
  p.wr->Write("xy", 2);
  p.wr->close();

  char buf[8] = {};
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_EQ(p.rd->Read(buf, 6), 2u);
  // It did not wait for the timeout: the closed pipe was detected
  EXPECT_LT(std::chrono::steady_clock::now() - t0, 1500ms);
  EXPECT_EQ(std::string(buf, 2), "xy");
  EXPECT_FALSE(p.rd->isOpen()) << "the read end is closed once the writer is gone";
}

TEST(CPipe, ReadWithoutTimeoutAfterWriterClosedReturnsZero)
{
  PipePair p;
  p.wr->close();
  char buf[4] = {};
  EXPECT_EQ(p.rd->Read(buf, 4), 0u);
}
#endif

TEST(CPipe, WriteEndPointFromSerializedString)
{
  PipePair p;
  const std::string serialized = p.wr->serialize();
  // Serializing hands the handle over: the original end-point is invalidated
  EXPECT_FALSE(p.wr->isOpen());
  CPipeWriteEndPoint reconstructed(serialized);
  EXPECT_TRUE(reconstructed.isOpen());
  EXPECT_EQ(reconstructed.Write("hey", 3), 3u);
  char buf[4] = {};
  EXPECT_EQ(p.rd->Read(buf, 3), 3u);
  EXPECT_EQ(std::string(buf, 3), "hey");
}

TEST(CPipe, EndPointsRejectTheWrongDirection)
{
  PipePair p;
  char buf[4] = {};
  EXPECT_ANY_THROW(p.wr->Read(buf, 1));
  EXPECT_ANY_THROW(p.rd->Write("x", 1));
}

TEST(CPipe, ReadingFromAClosedPipeThrows)
{
  PipePair p;
  p.rd->close();
  EXPECT_FALSE(p.rd->isOpen());
  char buf[4] = {};
  EXPECT_ANY_THROW(p.rd->Read(buf, 1));
  // Closing twice is harmless:
  EXPECT_NO_THROW(p.rd->close());
}
