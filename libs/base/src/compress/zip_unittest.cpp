/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/compress.h>
#include <mrpt/math/ops_vectors.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


TEST(Compress, DataBlockGZ)
{
	const size_t N = 20000;

	// Create a random "input" data vector
	vector_byte   in_data(N);
	for (size_t i=0;i<N;i++)
		in_data[i] = static_cast<uint8_t>(i);  // We need low entropy for compression to have something to do!

	// Compress it:
	vector_byte compress_data;

//	cout << "[test_compress_main] Invoking compress_gz_data_block" << endl;

	if (!mrpt::compress::zip::compress_gz_data_block(in_data,compress_data))
		GTEST_FAIL() << "Error in compress_gz_data_block\n";

//	cout << "Compressed gz-data: " << N << " -> " << compress_data.size() << " bytes." << endl;

	vector_byte recovered_data;
	if (!mrpt::compress::zip::decompress_gz_data_block(compress_data, recovered_data))
		GTEST_FAIL() << "Error in decompress_gz_data_block\n";

//	cout << "Decompressed data: " << recovered_data.size() << " bytes." << endl;

	uint8_t err = mrpt::math::sum( recovered_data - in_data );

	EXPECT_EQ(0, err ) << "Differences after compressing & decompressing with GZ\n";
}

