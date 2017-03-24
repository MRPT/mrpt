/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/core_defs.h>
#include <mrpt/utils/bits.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace std;

// Load data from constant file and check for exact match.
TEST(bits, reverseBytes)
{
	{
		uint8_t val   = 0x88, val_r;
		const uint8_t val_r_ok = 0x88;

		mrpt::utils::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::utils::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		uint16_t val   = 0x1122, val_r;
		const uint16_t val_r_ok = 0x2211;

		mrpt::utils::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::utils::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		uint32_t val   = UINT32_C(0x11223344), val_r;
		const uint32_t val_r_ok = UINT32_C(0x44332211);

		mrpt::utils::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::utils::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		uint64_t val   = UINT64_C( 0x1122334455667788 ), val_r;
		const uint64_t val_r_ok = UINT64_C( 0x8877665544332211 );

		mrpt::utils::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::utils::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
}
