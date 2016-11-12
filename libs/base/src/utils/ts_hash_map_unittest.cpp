/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/ts_hash_map.h>
#include <gtest/gtest.h>

template <typename T>
void simple_test_hash_string()
{
	T h1, h2;
	mrpt::utils::reduced_hash("prueba1", h1);
	mrpt::utils::reduced_hash("prueba2", h2);
	EXPECT_NE(h1, h2);
}

TEST(ts_hash_map, string_hash_u8) {
	simple_test_hash_string<uint8_t>();
}
TEST(ts_hash_map, string_hash_u16) {
	simple_test_hash_string<uint16_t>();
}
TEST(ts_hash_map, string_hash_u32) {
	simple_test_hash_string<uint32_t>();
}
TEST(ts_hash_map, string_hash_u64) {
	simple_test_hash_string<uint64_t>();
}

