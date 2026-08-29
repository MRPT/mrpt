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

#include <string>
#include <vector>

using mrpt::io::CMemoryStream;
using mrpt::io::CStream;

namespace
{
/** Returns everything written so far into `s`, as text. */
std::string writtenText(CMemoryStream& s)
{
  return {
      static_cast<const char*>(s.getRawBufferData()), static_cast<size_t>(s.getTotalBytesCount())};
}
}  // namespace

TEST(CStream, printf)
{
  CMemoryStream s;
  EXPECT_EQ(s.printf("%s=%i", "answer", 42), 9);
  EXPECT_EQ(writtenText(s), "answer=42");
}

TEST(CStream, printfWithNullFormatThrows)
{
  CMemoryStream s;
  EXPECT_ANY_THROW(s.printf(nullptr));
}

TEST(CStream, printfGrowsItsInternalBufferForLongOutputs)
{
  // The implementation starts with a 1 kB scratch buffer and doubles it until
  // the formatted text fits:
  CMemoryStream s;
  const std::string longArg(5000, 'x');
  EXPECT_EQ(s.printf("%s", longArg.c_str()), 5000);
  EXPECT_EQ(writtenText(s), longArg);
}

TEST(CStream, printfVector)
{
  {
    CMemoryStream s;
    const std::vector<int> v = {1, 2, 3};
    s.printf_vector("%i", v);
    EXPECT_EQ(writtenText(s), "[1,2,3]");
  }
  {
    // A custom separator, and the single-element case (no separator at all):
    CMemoryStream s;
    const std::vector<int> v = {7};
    s.printf_vector("%i", v, ';');
    EXPECT_EQ(writtenText(s), "[7]");
  }
  {
    CMemoryStream s;
    const std::vector<double> v = {1.5, 2.5};
    s.printf_vector("%.1f", v, ';');
    EXPECT_EQ(writtenText(s), "[1.5;2.5]");
  }
  {
    CMemoryStream s;
    const std::vector<int> v;
    s.printf_vector("%i", v);
    EXPECT_EQ(writtenText(s), "[]");
  }
}

TEST(CStream, getline)
{
  CMemoryStream s;
  const std::string data = "first\r\nsecond\nthird";
  s.Write(data.data(), data.size());
  s.Seek(0);

  std::string line;
  EXPECT_TRUE(s.getline(line));
  EXPECT_EQ(line, "first");  // the '\r' is ignored

  EXPECT_TRUE(s.getline(line));
  EXPECT_EQ(line, "second");

  // The last line has no terminator: reported as end-of-stream, and whatever
  // was read stays in the output string:
  EXPECT_FALSE(s.getline(line));
  EXPECT_EQ(line, "third");
}

TEST(CStream, getlineOnAnEmptyStream)
{
  CMemoryStream s;
  std::string line = "stale";
  EXPECT_FALSE(s.getline(line));
  EXPECT_TRUE(line.empty());
}

TEST(CStream, readBufferImmediateFallsBackToRead)
{
  CMemoryStream s;
  const std::string data = "abcdef";
  s.Write(data.data(), data.size());
  s.Seek(0);

  char buf[6] = {};
  EXPECT_EQ(s.ReadBufferImmediate(buf, 6), 6U);
  EXPECT_EQ(std::string(buf, 6), data);
}
