/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/reverse_bytes.h>
#include <mrpt/core/bit_cast.h>
#include <gtest/gtest.h>

// Load data from constant file and check for exact match.
TEST(bits, reverseBytes)
{
	{
		uint8_t val = 0x88, val_r;
		const uint8_t val_r_ok = 0x88;

		mrpt::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		int8_t val = 0x66, val_r;
		const int8_t val_r_ok = 0x66;

		mrpt::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		uint16_t val = 0x1122, val_r;
		const uint16_t val_r_ok = 0x2211;

		mrpt::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		int16_t val = 0x1122, val_r;
		const int16_t val_r_ok = 0x2211;

		mrpt::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		uint32_t val = UINT32_C(0x11223344), val_r;
		const uint32_t val_r_ok = UINT32_C(0x44332211);

		mrpt::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		int32_t val = INT32_C(0x11223344), val_r;
		const int32_t val_r_ok = INT32_C(0x44332211);

		mrpt::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		uint64_t val = UINT64_C(0x1122334455667788), val_r;
		const uint64_t val_r_ok = UINT64_C(0x8877665544332211);

		mrpt::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		int64_t val = INT64_C(0x1122334455667766), val_r;
		const int64_t val_r_ok = INT64_C(0x6677665544332211);

		mrpt::reverseBytes(val, val_r);
		EXPECT_EQ(val_r, val_r_ok);
		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, val_r_ok);
	}
	{
		bool valTrue = true;
		bool valFalse = false;
		mrpt::reverseBytesInPlace(valTrue);
		mrpt::reverseBytesInPlace(valFalse);
		EXPECT_TRUE(valTrue);
		EXPECT_FALSE(valFalse);
	}
	{
		uint8_t val = UINT8_C(0xc1);

		mrpt::reverseBytesInPlace(val);
		EXPECT_EQ(val, 0xc1);
	}
	{
		// 1.0 == 0x3F800000
		float val = 1;
		const uint32_t val_r_ok = UINT32_C(0x803f);

		mrpt::reverseBytesInPlace(val);
		auto val_check = bit_cast<uint32_t>(val);
		EXPECT_EQ(val_check, val_r_ok);
	}
	{
		// 1.0 == 0x3ff0000000000000
		double val = 1;
		const uint64_t val_r_ok = UINT32_C(0xf03f);

		mrpt::reverseBytesInPlace(val);
		auto val_check = bit_cast<uint64_t>(val);
		EXPECT_EQ(val_check, val_r_ok);
	}
}
