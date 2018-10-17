/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/core/format.h>
#include <gtest/gtest.h>
#include <random>
#include <algorithm>  // std::equal

// Defined in tests/test_main.cpp
namespace mrpt
{
extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
}

const size_t tst_data_len = 1000U;

// Generate random data:
void generate_test_data(std::vector<uint8_t>& tst_data)
{
	const unsigned int random_seed = 123U;
	std::mt19937 mersenne_engine{random_seed};
	// MSVC C++11 library enforces use of `unsigned short` as smallest type
	std::uniform_int_distribution<unsigned short> dist{0, 3};  // low entropy
	auto gen = [&dist, &mersenne_engine]() { return dist(mersenne_engine); };
	// auto gen = []() { return 1; };

	tst_data.resize(tst_data_len);
	std::generate(std::begin(tst_data), std::end(tst_data), gen);
}

TEST(CFileGZStreams, readwriteTmpFileCompressed)
{
	std::vector<uint8_t> tst_data;
	generate_test_data(tst_data);

	// Test for all compress levels:
	for (int compress_level = 1; compress_level <= 9; compress_level++)
	{
		const std::string fil = mrpt::system::getTempFileName() +
								std::string("_") +
								std::to_string(compress_level);
		// Write:
		{
			mrpt::io::CFileGZOutputStream fil_out;
			const bool open_ok = fil_out.open(fil, compress_level);
			EXPECT_TRUE(open_ok);

			const size_t wr_count = fil_out.Write(&tst_data[0], tst_data_len);
			EXPECT_EQ(wr_count, tst_data_len);
		}
		// Read:
		{
			mrpt::io::CFileGZInputStream fil_in;
			const bool open_ok = fil_in.open(fil);
			EXPECT_TRUE(open_ok);

			uint8_t rd_buf[tst_data_len + 5];
			const size_t rd_count = fil_in.Read(rd_buf, tst_data_len);
			EXPECT_EQ(rd_count, tst_data_len);

			EXPECT_TRUE(std::equal(
				std::begin(tst_data), std::end(tst_data), std::begin(rd_buf)));
		}
	}
}

TEST(CFileGZStreams, compareWithTestGZFiles)
{
	std::vector<uint8_t> tst_data;
	generate_test_data(tst_data);

	// Test for all compress levels:
	for (int compress_level = 1; compress_level <= 9; compress_level++)
	{
		const std::string fil = mrpt::format(
			"%s/tests/gz-tests/%i.gz",
			mrpt::MRPT_GLOBAL_UNITTEST_SRC_DIR.c_str(), compress_level);
		if (!mrpt::system::fileExists(fil))
		{
			GTEST_FAIL() << "ERROR: test due to missing file: " << fil << "\n";
			continue;
		}
		mrpt::io::CFileGZInputStream fil_in;
		const bool open_ok = fil_in.open(fil);
		EXPECT_TRUE(open_ok);

		uint8_t rd_buf[tst_data_len + 5];
		const size_t rd_count = fil_in.Read(rd_buf, tst_data_len);
		EXPECT_EQ(rd_count, tst_data_len);

		EXPECT_TRUE(std::equal(
			std::begin(tst_data), std::end(tst_data), std::begin(rd_buf)))
			<< " compress_level:" << compress_level;
	}
}
