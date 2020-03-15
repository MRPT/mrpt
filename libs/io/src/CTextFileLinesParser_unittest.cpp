/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

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
	EXPECT_EQ(parser.getCurrentLineNumber(), 1);
	EXPECT_EQ(line, "1st line");

	ret = parser.getNextLine(line);
	EXPECT_TRUE(ret);
	EXPECT_EQ(parser.getCurrentLineNumber(), 2);
	EXPECT_EQ(line, "2nd line");

	ret = parser.getNextLine(line);
	EXPECT_TRUE(ret);
	EXPECT_EQ(parser.getCurrentLineNumber(), 4);

	EXPECT_EQ(line, "3rd line");

	// EOF:
	EXPECT_FALSE(parser.getNextLine(line));
}
