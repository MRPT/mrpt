/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <mrpt/containers/circular_buffer.h>
#include <mrpt/core/common.h>
#include <mrpt/random.h>
#include <array>

#include <gtest/gtest.h>

template class mrpt::CTraitsTest<mrpt::containers::circular_buffer<char>>;

using cb_t = int;

TEST(circular_buffer_tests, EmptyPop)
{
	mrpt::containers::circular_buffer<cb_t> cb(10);
	cb_t ret;
	EXPECT_THROW(cb.pop(ret), std::exception);
}
TEST(circular_buffer_tests, EmptyPopAfterPushes)
{
	constexpr size_t LEN = 20;
	mrpt::containers::circular_buffer<cb_t> cb(LEN);
	for (size_t nWr = 0; nWr < LEN; nWr++)
	{
		for (size_t i = 0; i < nWr; i++) cb.push(12);
		cb_t ret;
		for (size_t i = 0; i < nWr; i++) cb.pop(ret);
		// The next one must fail:
		EXPECT_THROW(cb.pop(ret), std::exception);
	}
}

TEST(circular_buffer_tests, RandomWriteAndPeek)
{
	constexpr size_t LEN = 20;
	mrpt::containers::circular_buffer<cb_t> cb(LEN);

	for (size_t iter = 0; iter < 1000; iter++)
	{
		const size_t nWr =
			mrpt::random::getRandomGenerator().drawUniform32bit() % LEN;
		for (size_t i = 0; i < nWr; i++) cb.push(i);
		cb_t ret;
		for (size_t i = 0; i < nWr; i++)
		{
			ret = cb.peek(i);
			EXPECT_EQ(ret, cb_t(i));
		}
		for (size_t i = 0; i < nWr; i++)
		{
			cb.pop(ret);
			EXPECT_EQ(ret, cb_t(i));
		}
	}
}
TEST(circular_buffer_tests, RandomWriteManyAndPeek)
{
	constexpr size_t LEN = 20;
	mrpt::containers::circular_buffer<cb_t> cb(LEN);
	std::vector<cb_t> dum_buf;

	for (size_t iter = 0; iter < 1000; iter++)
	{
		const size_t nWr =
			1 +
			mrpt::random::getRandomGenerator().drawUniform32bit() % (LEN - 1);
		dum_buf.resize(nWr);
		cb.push_many(&dum_buf[0], nWr);
		cb_t ret;
		if (iter % 2)
		{
			for (size_t i = 0; i < nWr; i++) ret = cb.peek(i);
			MRPT_UNUSED_PARAM(ret);
		}
		else
		{
			cb.peek_many(&dum_buf[0], nWr);
		}
		if (iter % 3)
		{
			for (size_t i = 0; i < nWr; i++) cb.pop(ret);
			MRPT_UNUSED_PARAM(ret);
		}
		else
		{
			cb.pop_many(&dum_buf[0], nWr);
		}
	}
}
TEST(circular_buffer_tests, RandomWriteAndPeekOverrun)
{
	constexpr size_t LEN = 20;
	mrpt::containers::circular_buffer<cb_t> cb(LEN);

	for (size_t iter = 0; iter < 100; iter++)
	{
		const size_t nWr =
			mrpt::random::getRandomGenerator().drawUniform32bit() % LEN;
		for (size_t i = 0; i < nWr; i++) cb.push(i);
		cb_t ret;
		for (unsigned k = 0; k < 5; k++)
		{
			EXPECT_ANY_THROW(ret = cb.peek(nWr + k););
		}
		for (size_t i = 0; i < nWr; i++) cb.pop(ret);
	}
}

TEST(circular_buffer_tests, Size)
{
	mrpt::containers::circular_buffer<cb_t> cb(10);
	for (size_t i = 0; i < cb.capacity() - 1; i++)
	{
		cb.push(0);
		EXPECT_EQ(cb.size(), i + 1);
	}
	EXPECT_ANY_THROW(cb.push(0));
	for (size_t i = 0; i < cb.capacity() - 1; i++)
	{
		cb.pop();
		EXPECT_EQ(cb.size(), cb.capacity() - 2 - i);
	}
}

template <typename T>
void impl_WritePeekCheck()
{
	constexpr T LEN = 20;
	mrpt::containers::circular_buffer<T> cb(LEN + 1);

	for (T i = 0; i < LEN; i++) cb.push(i);

	std::array<T, LEN> peek_vals;
	cb.peek_many(&peek_vals[0], LEN);

	for (T i = 0; i < LEN; i++)
		EXPECT_EQ(static_cast<int>(peek_vals[i]), static_cast<int>(i));
}

TEST(circular_buffer_tests, WritePeekCheck_uint8_t)
{
	impl_WritePeekCheck<uint8_t>();
}
TEST(circular_buffer_tests, WritePeekCheck_uint16_t)
{
	impl_WritePeekCheck<uint16_t>();
}
TEST(circular_buffer_tests, WritePeekCheck_uint32_t)
{
	impl_WritePeekCheck<uint32_t>();
}
TEST(circular_buffer_tests, WritePeekCheck_uint64_t)
{
	impl_WritePeekCheck<uint64_t>();
}
