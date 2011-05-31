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
using namespace mrpt::random;
using namespace std;


// Load data from constant file and check for exact match.
TEST(Base64, RandomEncDec)
{
	for (size_t seed=0;seed<500;seed++)
	{
		randomGenerator.randomize(seed);

		const size_t block_len = randomGenerator.drawUniform32bit() % 567;

		vector_byte  myData(block_len);
		for (size_t n=0;n<block_len;n++)
			myData[n] = static_cast<uint8_t>(randomGenerator.drawUniform32bit());

		std::string myStr;
		mrpt::system::encodeBase64(myData,myStr);

		vector_byte outData;
		if (!mrpt::system::decodeBase64(myStr,outData))
			GTEST_FAIL() << "Error decoding the just encoded data!\n";

		// Compare data:
		EXPECT_EQ(outData.size(),myData.size());

		if (!myData.empty())
		{
			EXPECT_TRUE(outData == myData ) << "64-decoded data does not match original data!!\n";
		}
	}
}

