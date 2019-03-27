/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixFixed.h>
#include <Eigen/Dense>

TEST(CMatrixFixed, CtorUninit)
{
	mrpt::math::CMatrixFixed<double, 2, 2> M(mrpt::math::UNINITIALIZED_MATRIX);
	// do nothing, just test that the ctor above compiles
	(void)M(0, 0);
}

TEST(CMatrixFixed, CtorAllZeros)
{
	mrpt::math::CMatrixFixed<double, 2, 2> M;
	for (int i = 0; i < 2; i++)
		for (int j = 0; j < 2; j++) EXPECT_EQ(M(i, j), .0);
}

TEST(CMatrixFixed, Identity)
{
	mrpt::math::CMatrixFixed<double, 3, 3> M;
	M.setIdentity();
	for (int i = 0; i < 3; i++) EXPECT_EQ(M(i, i), 1.0);
}

TEST(CMatrixFixed, asStr)
{
	mrpt::math::CMatrixFixed<double, 2, 2> M;
	M.setIdentity();
	EXPECT_EQ(std::string("1 0\n0 1"), M.asStr());
}

TEST(CMatrixFixed, GetSetEigen)
{
	{
		mrpt::math::CMatrixFixed<double, 3, 3> M;
		auto em = M.asEigen();
		em.setIdentity();
		for (int i = 0; i < 3; i++) EXPECT_EQ(M(i, i), 1.0);
	}
	{
		mrpt::math::CMatrixFixed<double, 3, 3> M;
		auto em = M.asEigen();
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
			{
				const auto n = ((i + 1) * 3) + (j * 1001);
				em(i, j) = n;
				EXPECT_NEAR(M(i, j), em(i, j), 1e-9)
					<< "(i,j)=(" << i << "," << j << ")\n";
			}
	}
}
