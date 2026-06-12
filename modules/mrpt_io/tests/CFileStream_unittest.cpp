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
#include <mrpt/io/CFileStream.h>
#include <mrpt/system/filesystem.h>

#include <cstdio>

using namespace mrpt::io;

TEST(CFileStream, OpenNonExistingForReadFails)
{
  const std::string fname = mrpt::system::getTempFileName() + "_does_not_exist";
  CFileStream f;
  EXPECT_FALSE(f.open(fname, fomRead));
  EXPECT_FALSE(f.fileOpenCorrectly());
}

TEST(CFileStream, WriteReadCycle)
{
  const std::string fname = mrpt::system::getTempFileName() + "_CFileStream_test";

  {
    CFileStream fout(fname, fomWrite);
    ASSERT_TRUE(fout.fileOpenCorrectly());

    const std::string data = "0123456789";
    EXPECT_EQ(fout.Write(data.data(), data.size()), data.size());
    EXPECT_EQ(fout.getPositionO(), data.size());
  }

  {
    CFileStream fin(fname, fomRead);
    ASSERT_TRUE(fin.fileOpenCorrectly());
    EXPECT_TRUE(fin.is_open());

    EXPECT_EQ(fin.getTotalBytesCount(), 10U);

    char buf[10];
    EXPECT_EQ(fin.Read(buf, 10), 10U);
    EXPECT_EQ(std::string(buf, 10), "0123456789");
    EXPECT_EQ(fin.getPositionI(), 10U);

    EXPECT_FALSE(fin.checkEOF());
    char extra = 0;
    fin.Read(&extra, 1);
    EXPECT_TRUE(fin.checkEOF());

    fin.clearError();
    EXPECT_FALSE(fin.checkEOF());

    // Seek back to the start and verify positions:
    EXPECT_EQ(fin.Seek(0, CStream::sFromBeginning), 0U);
    EXPECT_EQ(fin.getPosition(), 0U);

    EXPECT_EQ(fin.Seek(2, CStream::sFromBeginning), 2U);
    char two = 0;
    EXPECT_EQ(fin.Read(&two, 1), 1U);
    EXPECT_EQ(two, '2');
  }

  std::remove(fname.c_str());
}

TEST(CFileStream, ReadLine)
{
  const std::string fname = mrpt::system::getTempFileName() + "_CFileStream_readline_test";

  {
    CFileStream fout(fname, fomWrite);
    ASSERT_TRUE(fout.fileOpenCorrectly());
    const std::string data = "line1\nline2\n";
    fout.Write(data.data(), data.size());
  }

  {
    CFileStream fin(fname, fomRead);
    ASSERT_TRUE(fin.fileOpenCorrectly());

    std::string line;
    EXPECT_TRUE(fin.readLine(line));
    EXPECT_EQ(line, "line1");

    EXPECT_TRUE(fin.readLine(line));
    EXPECT_EQ(line, "line2");

    // No more lines:
    EXPECT_FALSE(fin.readLine(line));
  }

  std::remove(fname.c_str());
}

TEST(CFileStream, ConstructorThrowsOnInvalidFile)
{
  const std::string fname = mrpt::system::getTempFileName() + "_does_not_exist_dir/file.txt";
  EXPECT_THROW(CFileStream(fname, fomRead), std::runtime_error);
}

TEST(CFileStream, UninitializedStreamReadWriteReturnZero)
{
  CFileStream f;
  EXPECT_FALSE(f.fileOpenCorrectly());
  EXPECT_EQ(f.getPosition(), 0U);
  EXPECT_EQ(f.getTotalBytesCount(), 0U);
  EXPECT_EQ(f.getPositionI(), 0U);
  EXPECT_EQ(f.getPositionO(), 0U);
  EXPECT_TRUE(f.checkEOF());
  EXPECT_EQ(f.Seek(0), 0U);

  char buf[4];
  EXPECT_EQ(f.Read(buf, 4), 0U);
  EXPECT_EQ(f.Write(buf, 4), 0U);
}
