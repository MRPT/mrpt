/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2025, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/system/string_utils.h>

#include <random>

using namespace mrpt;
using namespace std;

// Load data from constant file and check for exact match.
TEST(Base64, RandomEncDec)
{
  for (unsigned int seed = 0; seed < 500; seed++)
  {
    std::mt19937 rng(seed);

    std::uniform_int_distribution<size_t> length_dist(0, 566);  // inclusive range
    const size_t block_len = length_dist(rng);

    std::uniform_int_distribution<uint32_t> byte_dist(0, 0xFF);  // 8-bit random values
    std::vector<uint8_t> myData(block_len);
    for (size_t n = 0; n < block_len; n++)
    {
      myData[n] = static_cast<uint8_t>(byte_dist(rng));
    }

    std::string myStr;
    mrpt::system::encodeBase64(myData, myStr);

    std::vector<uint8_t> outData;
    if (!mrpt::system::decodeBase64(myStr, outData))
    {
      GTEST_FAIL() << "Error decoding the just encoded data!\n";
    }

    // Compare data:
    EXPECT_EQ(outData.size(), myData.size());

    if (!myData.empty())
    {
      EXPECT_TRUE(outData == myData) << "64-decoded data does not match original data!!\n";
    }
  }
}
