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

using namespace mrpt::io;

TEST(CPipe, WriteReadRoundTrip)
{
  std::unique_ptr<CPipeReadEndPoint> readPipe;
  std::unique_ptr<CPipeWriteEndPoint> writePipe;
  CPipe::createPipe(readPipe, writePipe);

  ASSERT_TRUE(readPipe);
  ASSERT_TRUE(writePipe);

  const std::string data = "Hello, pipe!";
  EXPECT_EQ(writePipe->Write(data.data(), data.size()), data.size());

  char buf[32] = {0};
  EXPECT_EQ(readPipe->Read(buf, data.size()), data.size());
  EXPECT_EQ(std::string(buf, data.size()), data);
}

TEST(CPipe, SerializeAndReconstructEndPoint)
{
  std::unique_ptr<CPipeReadEndPoint> readPipe;
  std::unique_ptr<CPipeWriteEndPoint> writePipe;
  CPipe::createPipe(readPipe, writePipe);

  const std::string serialized = readPipe->serialize();
  EXPECT_FALSE(serialized.empty());

  CPipeReadEndPoint reconstructed(serialized);

  const std::string data = "abc";
  EXPECT_EQ(writePipe->Write(data.data(), data.size()), data.size());

  char buf[8] = {0};
  EXPECT_EQ(reconstructed.Read(buf, data.size()), data.size());
  EXPECT_EQ(std::string(buf, data.size()), data);
}

TEST(CPipe, SeekAndCountsAreNoOps)
{
  std::unique_ptr<CPipeReadEndPoint> readPipe;
  std::unique_ptr<CPipeWriteEndPoint> writePipe;
  CPipe::createPipe(readPipe, writePipe);

  EXPECT_EQ(readPipe->Seek(0), 0U);
  EXPECT_EQ(readPipe->getTotalBytesCount(), 0U);
  EXPECT_EQ(readPipe->getPosition(), 0U);
}

TEST(CPipe, SerializeOnClosedPipeThrows)
{
  std::unique_ptr<CPipeReadEndPoint> readPipe;
  std::unique_ptr<CPipeWriteEndPoint> writePipe;
  CPipe::createPipe(readPipe, writePipe);

  readPipe->close();
  EXPECT_THROW(readPipe->serialize(), std::exception);
}

TEST(CPipe, InvalidSerializedStringThrows)
{
  EXPECT_THROW(CPipeReadEndPoint(std::string("not-a-number")), std::exception);
}
