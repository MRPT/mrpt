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
#include <mrpt/core/common.h>
#include <mrpt/io/CTextFileLinesParser.h>

#include <sstream>

TEST(CTextFileLinesParser, parse)
{
  std::stringstream ss;
  ss << "1st line\n"
        "2nd line\n"
        "# comment\n"
        "3rd line";
  ss.seekg(0);  // rewind

  mrpt::io::CTextFileLinesParser parser(ss);
  parser.enableCommentFilters(true, true, true);

  std::string line;
  bool ret = parser.getNextLine(line);
  EXPECT_TRUE(ret);
  EXPECT_EQ(parser.getCurrentLineNumber(), 1U);
  EXPECT_EQ(line, "1st line");

  ret = parser.getNextLine(line);
  EXPECT_TRUE(ret);
  EXPECT_EQ(parser.getCurrentLineNumber(), 2U);
  EXPECT_EQ(line, "2nd line");

  ret = parser.getNextLine(line);
  EXPECT_TRUE(ret);
  EXPECT_EQ(parser.getCurrentLineNumber(), 4U);

  EXPECT_EQ(line, "3rd line");

  // EOF:
  EXPECT_FALSE(parser.getNextLine(line));
}
