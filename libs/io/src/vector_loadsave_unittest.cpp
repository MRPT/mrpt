/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/format.h>
#include <mrpt/io/vector_loadsave.h>
#include <test_mrpt_common.h>

#include <regex>

TEST(file_get_contents, readTestFile)
{
	const std::string fil = mrpt::format(
		"%s/tests/sample_text_file.txt", mrpt::UNITTEST_BASEDIR().c_str());

	std::string contents = mrpt::io::file_get_contents(fil);
	contents = std::regex_replace(contents, std::regex("\r\n"), "\n");

	const std::string expectedContents = "0\n1\n2\n3\nhello\nworld!\n";

	EXPECT_EQ(contents, expectedContents);
}

TEST(file_get_contents, throwOnError)
{
	const std::string fil = mrpt::format(
		"%s/tests/sample_text_file_bis.txt", mrpt::UNITTEST_BASEDIR().c_str());

	EXPECT_ANY_THROW(mrpt::io::file_get_contents(fil));
}
