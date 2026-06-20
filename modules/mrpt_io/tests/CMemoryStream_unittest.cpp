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

TEST(CMemoryStream, readwrite)
{
  mrpt::io::CMemoryStream buf;

  buf.Write("1234567890", 10);
  EXPECT_EQ(buf.getPosition(), 10U);

  buf.Write("123456789", 9);
  EXPECT_EQ(buf.getPosition(), 19U);

  buf.Seek(0);
  EXPECT_EQ(buf.getPosition(), 0U);

  char r[100];
  auto nRead = buf.Read(r, 1);

  EXPECT_EQ(nRead, 1U);
  EXPECT_EQ(r[0], '1');

  nRead = buf.Read(&r[1], 100);

  EXPECT_EQ(nRead, 18U);
  EXPECT_EQ(r[18], '9');
}
