/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

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
