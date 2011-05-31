/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::slam;
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

