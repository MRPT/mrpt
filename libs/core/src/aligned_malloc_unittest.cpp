/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2023, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/aligned_allocator.h>

template <
	std::size_t alignment, typename T,
	typename = std::enable_if_t<std::is_pointer<T>::value>>
bool my_is_aligned(T ptr)
{
	return alignment == 0 ||
		reinterpret_cast<std::size_t>(ptr) % alignment == 0;
}

TEST(aligned_allocator, aligned_malloc)
{
	{
		void* p = mrpt::aligned_malloc(100, 4);
		EXPECT_TRUE(my_is_aligned<4>(p));
		mrpt::aligned_free(p);
	}
	{
		void* p = mrpt::aligned_malloc(100, 8);
		EXPECT_TRUE(my_is_aligned<8>(p));
		mrpt::aligned_free(p);
	}
	{
		void* p = mrpt::aligned_malloc(100, 16);
		EXPECT_TRUE(my_is_aligned<16>(p));
		mrpt::aligned_free(p);
	}
	{
		void* p = mrpt::aligned_malloc(100, 32);
		EXPECT_TRUE(my_is_aligned<32>(p));
		mrpt::aligned_free(p);
	}
}

TEST(aligned_allocator, aligned_calloc)
{
	void* p = mrpt::aligned_calloc(100, 32);
	EXPECT_TRUE(my_is_aligned<32>(p));
	const uint8_t* ptr = reinterpret_cast<uint8_t*>(p);
	for (int i = 0; i < 100; i++)
		EXPECT_EQ(ptr[i], 0);

	mrpt::aligned_free(p);
}
