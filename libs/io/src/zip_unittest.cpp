/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/io/zip.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::io;
using namespace std;

TEST(Compress, DataBlockGZ)
{
	const size_t N = 20000;

	// Create a random "input" data vector
	std::vector<uint8_t> in_data(N);
	for (size_t i = 0; i < N; i++)
		in_data[i] = static_cast<uint8_t>(
			i);  // We need low entropy for compression to have something to do!

	// Compress it:
	std::vector<uint8_t> compress_data;

	//	cout << "[test_compress_main] Invoking compress_gz_data_block" << endl;

	if (!mrpt::io::zip::compress_gz_data_block(in_data, compress_data))
		GTEST_FAIL() << "Error in compress_gz_data_block\n";

	//	cout << "Compressed gz-data: " << N << " -> " << compress_data.size() <<
	//" bytes." << endl;

	std::vector<uint8_t> recovered_data;
	if (!mrpt::io::zip::decompress_gz_data_block(compress_data, recovered_data))
		GTEST_FAIL() << "Error in decompress_gz_data_block\n";

	//	cout << "Decompressed data: " << recovered_data.size() << " bytes." <<
	// endl;

	EXPECT_EQ(in_data.size(), recovered_data.size());
	if (in_data.size() == recovered_data.size())
	{
		bool all_eq = true;
		for (size_t i = 0; i < in_data.size(); i++)
			if (in_data[i] != recovered_data[i]) all_eq = false;
		EXPECT_TRUE(all_eq) << "Mismatch after compressing/decompressing";
	}
}
