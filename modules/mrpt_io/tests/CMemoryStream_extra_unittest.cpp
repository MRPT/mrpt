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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/system/filesystem.h>

#include <array>
#include <cstdio>
#include <string>

using mrpt::io::CMemoryStream;
using mrpt::io::CStream;

TEST(CMemoryStream, constructFromExistingBuffer)
{
  const std::string data = "abcdef";
  CMemoryStream s(data.data(), data.size());

  EXPECT_EQ(s.getTotalBytesCount(), data.size());

  s.Seek(0);
  std::array<char, 6> buf{};
  EXPECT_EQ(s.Read(buf.data(), buf.size()), data.size());
  EXPECT_EQ(std::string(buf.data(), buf.size()), data);
}

TEST(CMemoryStream, constructFromNullBufferThrows) { EXPECT_ANY_THROW(CMemoryStream(nullptr, 10)); }

TEST(CMemoryStream, seekFromBeginningCurrentAndEnd)
{
  CMemoryStream s;
  const std::string data = "0123456789";
  s.Write(data.data(), data.size());

  EXPECT_EQ(s.Seek(3, CStream::sFromBeginning), 3U);
  EXPECT_EQ(s.getPosition(), 3U);

  EXPECT_EQ(s.Seek(2, CStream::sFromCurrent), 5U);
  EXPECT_EQ(s.Seek(-1, CStream::sFromCurrent), 4U);

  // Offsets from the end are negative, per the CStream contract:
  EXPECT_EQ(s.Seek(-4, CStream::sFromEnd), 6U);
  char c = 0;
  EXPECT_EQ(s.Read(&c, 1), 1U);
  EXPECT_EQ(c, '6');

  // Seeking to the very end is valid and leaves nothing to read:
  EXPECT_EQ(s.Seek(0, CStream::sFromEnd), data.size());
  EXPECT_EQ(s.Read(&c, 1), 0U);
}

TEST(CMemoryStream, seekClampsToTheValidRange)
{
  CMemoryStream s;
  const std::string data = "0123456789";
  s.Write(data.data(), data.size());

  EXPECT_EQ(s.Seek(1000, CStream::sFromBeginning), data.size());
  EXPECT_EQ(s.Seek(-1000, CStream::sFromBeginning), 0U);
  EXPECT_EQ(s.Seek(-1000, CStream::sFromCurrent), 0U);
}

TEST(CMemoryStream, seekOnAnEmptyStream)
{
  CMemoryStream s;
  EXPECT_EQ(s.Seek(0), 0U);
  EXPECT_EQ(s.Seek(0, CStream::sFromEnd), 0U);
  EXPECT_EQ(s.getPosition(), 0U);

  // The stream is still usable after seeking on it while empty:
  const std::string data = "hi";
  EXPECT_EQ(s.Write(data.data(), data.size()), data.size());
  EXPECT_EQ(s.getTotalBytesCount(), data.size());
}

TEST(CMemoryStream, seekThenWriteOverwritesInPlace)
{
  CMemoryStream s;
  const std::string data = "0123456789";
  s.Write(data.data(), data.size());

  s.Seek(2);
  s.Write("XY", 2);
  EXPECT_EQ(s.getTotalBytesCount(), data.size());

  s.Seek(0);
  std::array<char, 10> buf{};
  s.Read(buf.data(), buf.size());
  EXPECT_EQ(std::string(buf.data(), buf.size()), "01XY456789");
}

TEST(CMemoryStream, clearReleasesTheBuffer)
{
  CMemoryStream s;
  s.Write("0123456789", 10);
  EXPECT_EQ(s.getTotalBytesCount(), 10U);

  s.clear();
  EXPECT_EQ(s.getTotalBytesCount(), 0U);
  EXPECT_EQ(s.getPosition(), 0U);
  EXPECT_EQ(s.getRawBufferData(), nullptr);
}

TEST(CMemoryStream, assignedMemoryIsReadOnly)
{
  const std::string data = "abcdef";

  CMemoryStream s;
  s.assignMemoryNotOwn(data.data(), data.size());

  EXPECT_EQ(s.getTotalBytesCount(), data.size());
  EXPECT_EQ(s.getRawBufferData(), data.data());

  std::array<char, 6> buf{};
  EXPECT_EQ(s.Read(buf.data(), buf.size()), data.size());
  EXPECT_EQ(std::string(buf.data(), buf.size()), data);

  // The block is not ours to grow, so writing into it is rejected:
  s.Seek(0);
  EXPECT_ANY_THROW(s.Write(data.data(), data.size()));

  // ...until it is released:
  s.clear();
  EXPECT_EQ(s.getTotalBytesCount(), 0U);
  EXPECT_EQ(s.Write(data.data(), data.size()), data.size());
}

TEST(CMemoryStream, writeWithANullBufferThrows)
{
  CMemoryStream s;
  EXPECT_ANY_THROW(s.Write(nullptr, 4));
}

TEST(CMemoryStream, setAllocBlockSize)
{
  CMemoryStream s;
  EXPECT_ANY_THROW(s.setAllocBlockSize(0));

  EXPECT_NO_THROW(s.setAllocBlockSize(8));
  s.Write("0123456789", 10);
  EXPECT_EQ(s.getTotalBytesCount(), 10U);
}

TEST(CMemoryStream, saveAndLoadBufferFromFile)
{
  const std::string fname = mrpt::system::getTempFileName() + "_CMemoryStream_test";
  const std::string data = "some binary-ish payload";

  {
    CMemoryStream s;
    s.Write(data.data(), data.size());
    EXPECT_TRUE(s.saveBufferToFile(fname));
  }
  {
    CMemoryStream s;
    EXPECT_TRUE(s.loadBufferFromFile(fname));
    EXPECT_EQ(s.getTotalBytesCount(), data.size());

    s.Seek(0);
    std::string readBack(data.size(), '\0');
    EXPECT_EQ(s.Read(readBack.data(), readBack.size()), data.size());
    EXPECT_EQ(readBack, data);
  }

  std::remove(fname.c_str());
}

TEST(CMemoryStream, loadBufferFromMissingFileFails)
{
  CMemoryStream s;
  EXPECT_FALSE(s.loadBufferFromFile(mrpt::system::getTempFileName() + "_does_not_exist"));
}

TEST(CMemoryStream, saveBufferToUnwritablePathFails)
{
  CMemoryStream s;
  s.Write("x", 1);
  EXPECT_FALSE(s.saveBufferToFile("/this/directory/does/not/exist/file.bin"));
}

TEST(CMemoryStream, streamDescription)
{
  CMemoryStream s;
  s.Write("0123456789", 10);

  const std::string d = s.getStreamDescription();
  EXPECT_NE(d.find("CMemoryStream"), std::string::npos);
  EXPECT_NE(d.find("bytesWritten=10"), std::string::npos);
}
