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
#include <mrpt/io/CFileOutputStream.h>
#include <mrpt/io/CTextFileLinesParser.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/system/filesystem.h>

#include <cstdio>
#include <sstream>
#include <string>

// ------------------------------ lazy_load_path -----------------------------

namespace
{
/** Restores the process-wide lazy-load base path when the test ends. */
class LazyLoadPathGuard
{
 public:
  LazyLoadPathGuard() : m_saved(mrpt::io::getLazyLoadPathBase()) {}
  ~LazyLoadPathGuard() { mrpt::io::setLazyLoadPathBase(m_saved); }

  LazyLoadPathGuard(const LazyLoadPathGuard&) = delete;
  LazyLoadPathGuard& operator=(const LazyLoadPathGuard&) = delete;
  LazyLoadPathGuard(LazyLoadPathGuard&&) = delete;
  LazyLoadPathGuard& operator=(LazyLoadPathGuard&&) = delete;

 private:
  std::string m_saved;
};
}  // namespace

TEST(lazy_load_path, defaultBaseIsTheCurrentDirectory)
{
  EXPECT_EQ(mrpt::io::getLazyLoadPathBase(), ".");
}

TEST(lazy_load_path, absolutePathsAreLeftUntouched)
{
  EXPECT_EQ(mrpt::io::lazy_load_absolute_path("/tmp/foo.bin"), "/tmp/foo.bin");
  // Windows-style drive letters count as absolute too:
  EXPECT_EQ(mrpt::io::lazy_load_absolute_path("C:\\data\\foo.bin"), "C:\\data\\foo.bin");
  EXPECT_EQ(mrpt::io::lazy_load_absolute_path("C:/data/foo.bin"), "C:/data/foo.bin");
}

TEST(lazy_load_path, relativePathsArePrefixedWithTheBase)
{
  const LazyLoadPathGuard guard;

  mrpt::io::setLazyLoadPathBase("/base/dir");
  EXPECT_EQ(mrpt::io::getLazyLoadPathBase(), "/base/dir");
  EXPECT_EQ(mrpt::io::lazy_load_absolute_path("foo.bin"), "/base/dir/foo.bin");

  // A separator already present in the base is not duplicated:
  mrpt::io::setLazyLoadPathBase("/base/dir/");
  EXPECT_EQ(mrpt::io::lazy_load_absolute_path("foo.bin"), "/base/dir/foo.bin");

  mrpt::io::setLazyLoadPathBase("C:\\base\\");
  EXPECT_EQ(mrpt::io::lazy_load_absolute_path("foo.bin"), "C:\\base\\foo.bin");
}

TEST(lazy_load_path, tooShortPathsAreRejected)
{
  EXPECT_ANY_THROW(mrpt::io::lazy_load_absolute_path("ab"));
}

// --------------------------- CTextFileLinesParser --------------------------

TEST(CTextFileLinesParser, openingAMissingFileThrows)
{
  EXPECT_ANY_THROW(
      mrpt::io::CTextFileLinesParser(mrpt::system::getTempFileName() + "_does_not_exist"));
}

TEST(CTextFileLinesParser, parseAFileAndRewind)
{
  const std::string fname = mrpt::system::getTempFileName() + "_lines_parser";
  {
    mrpt::io::CFileOutputStream f(fname);
    const std::string data = "  alpha  \n\n% matlab comment\nbeta\n";
    f.Write(data.data(), data.size());
  }

  mrpt::io::CTextFileLinesParser parser(fname);

  std::string line;
  ASSERT_TRUE(parser.getNextLine(line));
  EXPECT_EQ(line, "alpha");  // surrounding blanks are trimmed
  ASSERT_TRUE(parser.getNextLine(line));
  EXPECT_EQ(line, "beta");
  EXPECT_FALSE(parser.getNextLine(line));
  EXPECT_TRUE(line.empty());

  parser.rewind();
  EXPECT_EQ(parser.getCurrentLineNumber(), 0U);
  ASSERT_TRUE(parser.getNextLine(line));
  EXPECT_EQ(line, "alpha");

  parser.close();
  std::remove(fname.c_str());
}

TEST(CTextFileLinesParser, commentFiltersCanBeDisabled)
{
  std::stringstream ss;
  ss << "# sh\n"
        "// c\n"
        "% matlab\n"
        "data\n";
  ss.seekg(0);

  mrpt::io::CTextFileLinesParser parser(ss);
  parser.enableCommentFilters(false, false, false);

  std::string line;
  ASSERT_TRUE(parser.getNextLine(line));
  EXPECT_EQ(line, "# sh");
  ASSERT_TRUE(parser.getNextLine(line));
  EXPECT_EQ(line, "// c");
  ASSERT_TRUE(parser.getNextLine(line));
  EXPECT_EQ(line, "% matlab");
  ASSERT_TRUE(parser.getNextLine(line));
  EXPECT_EQ(line, "data");
  EXPECT_FALSE(parser.getNextLine(line));
}

TEST(CTextFileLinesParser, getNextLineAsAStringStream)
{
  std::stringstream ss;
  ss << "10 20 30\n";
  ss.seekg(0);

  mrpt::io::CTextFileLinesParser parser(ss);

  std::istringstream buf;
  ASSERT_TRUE(parser.getNextLine(buf));

  int a = 0;
  int b = 0;
  int c = 0;
  buf >> a >> b >> c;
  EXPECT_EQ(a, 10);
  EXPECT_EQ(b, 20);
  EXPECT_EQ(c, 30);
}

TEST(CTextFileLinesParser, closingTwiceIsHarmless)
{
  std::stringstream ss;
  ss << "x\n";
  ss.seekg(0);

  mrpt::io::CTextFileLinesParser parser(ss);
  EXPECT_NO_THROW(parser.close());
  EXPECT_NO_THROW(parser.close());
}
