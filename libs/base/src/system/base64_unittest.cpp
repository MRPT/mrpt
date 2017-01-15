/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/system/string_utils.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>

using namespace mrpt;
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

