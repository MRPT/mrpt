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
#include <mrpt/system/string_utils.h>

TEST(string_utils, firstNLines)
{
  const std::string s = "1\n2\n3\n4\n";

  EXPECT_EQ(mrpt::system::firstNLines(s, 1), "1");
  EXPECT_EQ(mrpt::system::firstNLines(s, 2), "1\n2");
  EXPECT_EQ(mrpt::system::firstNLines(s, 3), "1\n2\n3");
  EXPECT_EQ(mrpt::system::firstNLines(s, 10), s);
}

TEST(string_utils, decodeUTF8)
{
  using mrpt::system::decodeUTF8;
  std::vector<uint16_t> out;

  decodeUTF8("a\xC3\xB1\xE2\x82\xAC", out);  // a, n-tilde, euro sign
  EXPECT_EQ(out, (std::vector<uint16_t>{0x61, 0xF1, 0x20AC}));

  // Invalid input is replaced by U+FFFD, and decoding always ends:
  decodeUTF8("x\xB0y", out);  // stray continuation byte (Latin-1 degree sign)
  EXPECT_EQ(out, (std::vector<uint16_t>{0x78, 0xFFFD, 0x79}));

  decodeUTF8("x\xF0\x9F\x98\x80y", out);  // 4-byte sequence: not in the BMP
  EXPECT_EQ(out, (std::vector<uint16_t>{0x78, 0xFFFD, 0x79}));

  decodeUTF8("x\xE2\x82", out);  // truncated sequence
  EXPECT_EQ(out, (std::vector<uint16_t>{0x78, 0xFFFD}));
}

TEST(string_utils, encodeUTF8)
{
  std::string s;
  mrpt::system::encodeUTF8({0x61, 0xF1, 0x20AC, 0xFFFD}, s);
  EXPECT_EQ(s, "a\xC3\xB1\xE2\x82\xAC\xEF\xBF\xBD");

  std::vector<uint16_t> back;
  mrpt::system::decodeUTF8(s, back);
  EXPECT_EQ(back, (std::vector<uint16_t>{0x61, 0xF1, 0x20AC, 0xFFFD}));
}
