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
#include <mrpt/io/CFileInputStream.h>
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/system/filesystem.h>

#include <array>
#include <cstdio>
#include <string>

using mrpt::io::CFileInputStream;
using mrpt::io::CFileOutputStream;
using mrpt::io::CStream;
using mrpt::io::OpenMode;

namespace
{
/** Writes `contents` into a fresh temporary file, and returns its path. */
std::string makeTempFileWith(const std::string& contents, const std::string& suffix)
{
  const std::string fname = mrpt::system::getTempFileName() + suffix;
  CFileOutputStream f(fname);
  if (!contents.empty())
  {
    f.Write(contents.data(), contents.size());
  }
  return fname;
}
}  // namespace

// --------------------------- CFileInputStream ------------------------------

TEST(CFileInputStream, constructorThrowsForAMissingFile)
{
  EXPECT_ANY_THROW(CFileInputStream(mrpt::system::getTempFileName() + "_does_not_exist"));
}

TEST(CFileInputStream, unopenedStreamIsInert)
{
  CFileInputStream f;
  EXPECT_FALSE(f.fileOpenCorrectly());
  EXPECT_TRUE(f.checkEOF());
  EXPECT_EQ(f.getPosition(), 0U);
  EXPECT_EQ(f.getTotalBytesCount(), 0U);
  EXPECT_EQ(f.Seek(10), 0U);

  std::array<char, 4> buf{};
  EXPECT_EQ(f.Read(buf.data(), buf.size()), 0U);

  std::string line = "stale";
  EXPECT_FALSE(f.readLine(line));
  EXPECT_TRUE(line.empty());

  EXPECT_NO_THROW(f.clearError());
  EXPECT_NO_THROW(f.close());
}

TEST(CFileInputStream, writingToAReadStreamThrows)
{
  const std::string fname = makeTempFileWith("data", "_cfis_write");

  CFileInputStream f(fname);
  ASSERT_TRUE(f.fileOpenCorrectly());
  EXPECT_ANY_THROW(f.Write("x", 1));

  std::remove(fname.c_str());
}

TEST(CFileInputStream, readSeekAndSize)
{
  const std::string data = "0123456789";
  const std::string fname = makeTempFileWith(data, "_cfis_read");

  CFileInputStream f(fname);
  ASSERT_TRUE(f.fileOpenCorrectly());
  EXPECT_EQ(f.getTotalBytesCount(), data.size());

  char c = 0;
  EXPECT_EQ(f.Seek(4, CStream::sFromBeginning), 4U);
  EXPECT_EQ(f.Read(&c, 1), 1U);
  EXPECT_EQ(c, '4');

  EXPECT_EQ(f.Seek(2, CStream::sFromCurrent), 7U);
  EXPECT_EQ(f.Read(&c, 1), 1U);
  EXPECT_EQ(c, '7');

  EXPECT_EQ(f.Seek(-1, CStream::sFromEnd), data.size() - 1);
  EXPECT_EQ(f.Read(&c, 1), 1U);
  EXPECT_EQ(c, '9');

  // Reading past the end fails, and is reported as EOF until cleared:
  EXPECT_EQ(f.Read(&c, 1), 0U);
  EXPECT_TRUE(f.checkEOF());
  f.clearError();
  EXPECT_FALSE(f.checkEOF());

  std::remove(fname.c_str());
}

TEST(CFileInputStream, readLine)
{
  const std::string fname = makeTempFileWith("alpha\nbeta\ngamma", "_cfis_lines");

  CFileInputStream f(fname);
  ASSERT_TRUE(f.fileOpenCorrectly());

  std::string line;
  EXPECT_TRUE(f.readLine(line));
  EXPECT_EQ(line, "alpha");
  EXPECT_TRUE(f.readLine(line));
  EXPECT_EQ(line, "beta");
  // The last line has no terminator, so it reads as an end-of-file:
  EXPECT_FALSE(f.readLine(line));

  std::remove(fname.c_str());
}

TEST(CFileInputStream, openCloseAndReopen)
{
  const std::string fname = makeTempFileWith("abc", "_cfis_reopen");

  CFileInputStream f;
  EXPECT_TRUE(f.open(fname));
  EXPECT_TRUE(f.fileOpenCorrectly());
  EXPECT_NE(f.getStreamDescription().find(fname), std::string::npos);

  f.close();
  EXPECT_FALSE(f.fileOpenCorrectly());

  EXPECT_FALSE(f.open(mrpt::system::getTempFileName() + "_does_not_exist"));

  std::remove(fname.c_str());
}

// --------------------------- CFileOutputStream -----------------------------

TEST(CFileOutputStream, constructorThrowsForAnUnwritablePath)
{
  EXPECT_ANY_THROW(CFileOutputStream("/this/directory/does/not/exist/file.bin"));
}

TEST(CFileOutputStream, unopenedStreamIsInert)
{
  CFileOutputStream f;
  EXPECT_FALSE(f.fileOpenCorrectly());
  EXPECT_EQ(f.getPosition(), 0U);
  EXPECT_EQ(f.getTotalBytesCount(), 0U);
  EXPECT_EQ(f.Seek(10), 0U);
  EXPECT_EQ(f.Write("x", 1), 0U);
  EXPECT_NO_THROW(f.close());
}

TEST(CFileOutputStream, readingFromAWriteStreamThrows)
{
  const std::string fname = mrpt::system::getTempFileName() + "_cfos_read";

  CFileOutputStream f(fname);
  ASSERT_TRUE(f.fileOpenCorrectly());

  std::array<char, 4> buf{};
  EXPECT_ANY_THROW(f.Read(buf.data(), buf.size()));

  std::remove(fname.c_str());
}

TEST(CFileOutputStream, writeSeekAndSize)
{
  const std::string fname = mrpt::system::getTempFileName() + "_cfos_write";

  {
    CFileOutputStream f(fname);
    ASSERT_TRUE(f.fileOpenCorrectly());
    EXPECT_NE(f.getStreamDescription().find(fname), std::string::npos);

    const std::string data = "0123456789";
    EXPECT_EQ(f.Write(data.data(), data.size()), data.size());
    EXPECT_EQ(f.getPosition(), data.size());
    EXPECT_EQ(f.getTotalBytesCount(), data.size());

    // Overwrite in place:
    EXPECT_EQ(f.Seek(2, CStream::sFromBeginning), 2U);
    f.Write("XY", 2);
    EXPECT_EQ(f.Seek(0, CStream::sFromEnd), data.size());
    EXPECT_EQ(f.Seek(-3, CStream::sFromCurrent), data.size() - 3);
  }

  CFileInputStream in(fname);
  std::array<char, 10> buf{};
  in.Read(buf.data(), buf.size());
  EXPECT_EQ(std::string(buf.data(), buf.size()), "01XY456789");

  std::remove(fname.c_str());
}

TEST(CFileOutputStream, truncateAndAppendModes)
{
  const std::string fname = mrpt::system::getTempFileName() + "_cfos_append";

  {
    CFileOutputStream f(fname);
    f.Write("AAA", 3);
  }
  {
    CFileOutputStream f(fname, OpenMode::APPEND);
    ASSERT_TRUE(f.fileOpenCorrectly());
    f.Write("BBB", 3);
  }
  {
    CFileInputStream in(fname);
    EXPECT_EQ(in.getTotalBytesCount(), 6U);
  }
  {
    // The default mode truncates:
    CFileOutputStream f(fname, OpenMode::TRUNCATE);
    f.Write("C", 1);
  }
  {
    CFileInputStream in(fname);
    EXPECT_EQ(in.getTotalBytesCount(), 1U);
  }

  std::remove(fname.c_str());
}

TEST(CFileOutputStream, openCloseAndReopen)
{
  const std::string fname = mrpt::system::getTempFileName() + "_cfos_reopen";

  CFileOutputStream f;
  EXPECT_TRUE(f.open(fname));
  EXPECT_TRUE(f.fileOpenCorrectly());
  f.close();
  EXPECT_FALSE(f.fileOpenCorrectly());

  EXPECT_FALSE(f.open("/this/directory/does/not/exist/file.bin"));

  std::remove(fname.c_str());
}
