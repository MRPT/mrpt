/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/circular_buffer.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>

typedef int cb_t;

TEST(circular_buffer_tests, EmptyPop)
{
	mrpt::utils::circular_buffer<cb_t> cb(10);
	try {
		cb_t ret;
		cb.pop(ret);
		GTEST_FAIL() << "Exception was expected but didn't happen!";
	} catch (std::exception &) {
		// OK
	}
}
TEST(circular_buffer_tests, EmptyPopAfterPushes)
{
	const size_t LEN = 20;
	mrpt::utils::circular_buffer<cb_t> cb(LEN);
	for (size_t nWr=0;nWr<LEN;nWr++) 
	{
		for (size_t i=0;i<nWr;i++) cb.push(12);
		cb_t ret;
		for (size_t i=0;i<nWr;i++) cb.pop(ret);
		// The next one must fail:
		try {
			cb.pop(ret);
			GTEST_FAIL() << "Exception was expected but didn't happen!";
		} catch (std::exception &) {
			// OK
		}
	}
}

TEST(circular_buffer_tests, RandomWriteAndPeek)
{
	const size_t LEN = 20;
	mrpt::utils::circular_buffer<cb_t> cb(LEN);

	for (size_t iter=0;iter<1000;iter++) 
	{
		const size_t nWr = mrpt::random::randomGenerator.drawUniform32bit() % LEN;
		for (size_t i=0;i<nWr;i++) cb.push(i);
		cb_t ret;
		for (size_t i=0;i<nWr;i++) { 
			ret=cb.peek(i);
			EXPECT_EQ(ret,cb_t(i));
		}
		for (size_t i=0;i<nWr;i++) {
			cb.pop(ret);
			EXPECT_EQ(ret,cb_t(i));
		}
	}
}
TEST(circular_buffer_tests, RandomWriteManyAndPeek)
{
	const size_t LEN = 20;
	mrpt::utils::circular_buffer<cb_t> cb(LEN);
	std::vector<cb_t> dum_buf;

	for (size_t iter=0;iter<1000;iter++) 
	{
		const size_t nWr = 1+mrpt::random::randomGenerator.drawUniform32bit() % (LEN-1);
		dum_buf.resize(nWr);
		cb.push_many(&dum_buf[0],nWr);
		cb_t ret;
		if (iter%1) {
			for (size_t i=0;i<nWr;i++) ret=cb.peek(i);
			MRPT_UNUSED_PARAM(ret);
		} else {
			cb.peek_many(&dum_buf[0],nWr);
		}
		cb.pop_many(&dum_buf[0],nWr);
	}
}
TEST(circular_buffer_tests, RandomWriteAndPeekOverrun)
{
	const size_t LEN = 20;
	mrpt::utils::circular_buffer<cb_t> cb(LEN);

	for (size_t iter=0;iter<100;iter++)
	{
		const size_t nWr = mrpt::random::randomGenerator.drawUniform32bit() % LEN;
		for (size_t i=0;i<nWr;i++) cb.push(i);
		cb_t ret;
		for (unsigned k=0;k<5;k++) {
			try {
				ret=cb.peek(nWr+k);
				GTEST_FAIL() << "Exception was expected but didn't happen!";
			} catch (std::exception &) {
				// OK
			}
		}
		for (size_t i=0;i<nWr;i++) cb.pop(ret);
	}
}

