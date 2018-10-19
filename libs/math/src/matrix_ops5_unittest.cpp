/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

// Note: Matrices unit tests have been split in different files since
// building them with eigen3 eats a lot of RAM and may be a problem while
// compiling in small systems.

#include <mrpt/math/CMatrixFixedNumeric.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::random;
using namespace std;

TEST(Matrices, loadFromArray)
{
	alignas(MRPT_MAX_ALIGN_BYTES)
		const double nums[3 * 4] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

	CMatrixFixedNumeric<double, 3, 4> mat;
	mat.loadFromArray(nums);

	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 4; c++) EXPECT_EQ(nums[4 * r + c], mat(r, c));
}

alignas(MRPT_MAX_ALIGN_BYTES) static double test_nums[3 * 4] = {
	1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

TEST(Matrices, CMatrixFixedNumeric_loadWithEigenMap)
{
	// Row major
	const CMatrixFixedNumeric<double, 3, 4> mat = Eigen::Map<
		CMatrixFixedNumeric<double, 3, 4>::Base, MRPT_MAX_ALIGN_BYTES>(
		test_nums);

	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 4; c++) EXPECT_EQ(test_nums[4 * r + c], mat(r, c));
}

TEST(Matrices, EigenMatrix_loadWithEigenMap)
{
	// Col major
	const Eigen::Matrix<double, 3, 4> mat =
		Eigen::Map<Eigen::Matrix<double, 3, 4>, MRPT_MAX_ALIGN_BYTES>(
			test_nums);

	for (int r = 0; r < 3; r++)  // Transposed!!
		for (int c = 0; c < 4; c++) EXPECT_EQ(test_nums[3 * c + r], mat(r, c));
}
