/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <Eigen/Dense>

TEST(CMatrixDynamic, GetSetEigen)
{
	{
		auto M = mrpt::math::CMatrixDynamic<double>::Identity(3);
		auto em = M.asEigen();
		em.setIdentity();
		for (int i = 0; i < 3; i++) EXPECT_EQ(M(i, i), 1.0);
	}
	{
		mrpt::math::CMatrixDynamic<double> M(3, 3);
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
TEST(CMatrixDynamic, asStr)
{
	auto M = mrpt::math::CMatrixDynamic<double>::Identity(2);
	M.setIdentity();
	EXPECT_EQ(std::string("1 0\n0 1"), M.asStr());
}

TEST(CMatrixDynamic, CtorFromArray)
{
	const double dat_R[] = {1., 2., 3., 4., 5., 6., 7., 8., 9.};
	mrpt::math::CMatrixDouble R(3, 3, dat_R);
	for (int r = 0; r < 3; r++)
	{
		for (int c = 0; c < 3; c++)
		{
			EXPECT_EQ(dat_R[c + r * 3], R(r, c))
				<< "(r,c)=(" << r << "," << c << ")\n";
		}
	}
}

// Added to run with valgrind to error checking.
TEST(CMatrixDynamic, Resizes)
{
	using mrpt::math::CMatrixDouble;
	{
		CMatrixDouble M(0, 0);
		M.resize(0, 0);
		M.resize(1, 1);
		M.resize(10, 10);
		M.resize(15, 10);
		M.resize(15, 12);
		M.resize(5, 12);
		M.resize(3, 2);
	}
	{
		CMatrixDouble M(40, 50);
		M.resize(3, 3);
	}
}
