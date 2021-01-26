/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/core/alignment_req.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/system/memory.h>

#include <Eigen/Dense>
#include <iostream>

TEST(EigenAlignment, PrintAlignment)
{
	// MRPT_MAX_ALIGN_BYTES from: <mrpt/core/alignment_req.h>
	// EIGEN_MAX_ALIGN_BYTES from eigen macros.h

	std::cout << "MRPT_MAX_ALIGN_BYTES         : " << MRPT_MAX_ALIGN_BYTES
			  << "\n"
				 "MRPT_MAX_STATIC_ALIGN_BYTES  : "
			  << MRPT_MAX_STATIC_ALIGN_BYTES
			  << "\n"
				 "EIGEN_MAX_ALIGN_BYTES        : "
			  << EIGEN_MAX_ALIGN_BYTES
			  << "\n"
				 "EIGEN_MAX_STATIC_ALIGN_BYTES : "
			  << EIGEN_MAX_STATIC_ALIGN_BYTES << "\n";

	EXPECT_GE(MRPT_MAX_ALIGN_BYTES, EIGEN_MAX_ALIGN_BYTES);
	EXPECT_GE(MRPT_MAX_STATIC_ALIGN_BYTES, EIGEN_MAX_STATIC_ALIGN_BYTES);
}

template <unsigned int INTER_SPACE>
struct Foo
{
	mrpt::math::CMatrixFixed<double, 3, 3> m1;

	std::array<uint8_t, INTER_SPACE> dummy_space;

	mrpt::math::CMatrixFixed<double, 3, 3> m2;
};

template <unsigned int INTER_SPACE>
void checkAlignedFoo(const Foo<INTER_SPACE>& d, const std::string& testName)
{
	EXPECT_TRUE(mrpt::system::is_aligned<EIGEN_MAX_ALIGN_BYTES>(&d.m1(0, 0)))
		<< "test: " << testName << "\nINTER_SPACE=" << INTER_SPACE
		<< "\nm1(0,0)=" << static_cast<const void*>(&d.m1(0, 0))
		<< "\nFailed against: EIGEN_MAX_ALIGN_BYTES=" << EIGEN_MAX_ALIGN_BYTES;
	EXPECT_TRUE(mrpt::system::is_aligned<EIGEN_MAX_ALIGN_BYTES>(&d.m2(0, 0)))
		<< "test: " << testName << "\nINTER_SPACE=" << INTER_SPACE
		<< "\nm2(0,0)=" << static_cast<const void*>(&d.m2(0, 0))
		<< "\nFailed against: EIGEN_MAX_ALIGN_BYTES=" << EIGEN_MAX_ALIGN_BYTES;
}

template <unsigned int INTER_SPACE>
void do_test_AlignedMemOnStack()
{
	Foo<INTER_SPACE> d;
	checkAlignedFoo(d, "do_test_AlignedMemOnStack");
}
template <unsigned int INTER_SPACE>
void do_test_AlignedMemOnStackUpTo()
{
	do_test_AlignedMemOnStack<INTER_SPACE>();
	if constexpr (INTER_SPACE > 0)
		do_test_AlignedMemOnStackUpTo<INTER_SPACE - 1>();
}

template <unsigned int INTER_SPACE>
void do_test_AlignedMemInSTL()
{
	std::map<int, Foo<INTER_SPACE>> d;
	for (int i = 0; i < 100; i++)
		d[i * 3 + i * i * 4 + 1];

	checkAlignedFoo(d.rbegin()->second, "do_test_AlignedMemInSTL");
}
template <unsigned int INTER_SPACE>
void do_test_AlignedMemInSTLUpTo()
{
	do_test_AlignedMemInSTL<INTER_SPACE>();
	if constexpr (INTER_SPACE > 0)
		do_test_AlignedMemInSTLUpTo<INTER_SPACE - 1>();
}

TEST(EigenAlignment, AlignedMemOnStack) { do_test_AlignedMemOnStackUpTo<20>(); }

TEST(EigenAlignment, AlignedMemInSTL) { do_test_AlignedMemInSTLUpTo<20>(); }
