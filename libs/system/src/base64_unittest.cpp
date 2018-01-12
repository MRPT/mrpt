/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/system/string_utils.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::random;
using namespace std;

// Load data from constant file and check for exact match.
TEST(Base64, RandomEncDec)
{
	for (unsigned int seed = 0; seed < 500; seed++)
	{
		getRandomGenerator().randomize(seed);

		const size_t block_len = getRandomGenerator().drawUniform32bit() % 567;

		std::vector<uint8_t> myData(block_len);
		for (size_t n = 0; n < block_len; n++)
			myData[n] =
				static_cast<uint8_t>(getRandomGenerator().drawUniform32bit());

		std::string myStr;
		mrpt::system::encodeBase64(myData, myStr);

		std::vector<uint8_t> outData;
		if (!mrpt::system::decodeBase64(myStr, outData))
			GTEST_FAIL() << "Error decoding the just encoded data!\n";

		// Compare data:
		EXPECT_EQ(outData.size(), myData.size());

		if (!myData.empty())
		{
			EXPECT_TRUE(outData == myData)
				<< "64-decoded data does not match original data!!\n";
		}
	}
}
