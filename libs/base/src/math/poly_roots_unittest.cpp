/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/math/poly_roots.h>
#include <mrpt/system/os.h>
#include <gtest/gtest.h>
#include <cmath>

using namespace std;

const double eps = 1e-9;

TEST(poly_roots,solve_poly2)
{
	// `a*x^2 + b*x + c = 0`
	// Table: 
	//     coefs (a,b,c)    num_real_roots   root1  root2
	double coefs_roots[][6] = {
		{ 1   ,-2     , 1    ,   2,   1.0,  1.0 },
		{ 1   , 0     ,-1    ,   2,  -1.0,  1.0 },
		{ 1   ,-1    ,-56   ,   2,  -7.0,  8.0 },
		{ 5.0 , 0    , 1    ,   0,     0,    0 },
		{ 2.0 , 0    , 0    ,   2,     0,    0 }
	};

	const unsigned int nTests = sizeof(coefs_roots)/sizeof(coefs_roots[0]);

	for (unsigned int i=0;i<nTests;i++)
	{
		const double a = coefs_roots[i][0], b = coefs_roots[i][1], c = coefs_roots[i][2];
		const int num_roots_good = static_cast<int>( coefs_roots[i][3] );
		const double r1_good = coefs_roots[i][4], r2_good = coefs_roots[i][5];

		double r1,r2;
		int num_roots = mrpt::math::solve_poly2(a,b,c, r1,r2);
		
		const std::string sTestStr = mrpt::format("\nSolving: %.02f * x^2 + %.02f * x + %.02f = 0\n", a,b,c);

		EXPECT_EQ(num_roots,num_roots_good);
		if (num_roots>=1) {
			EXPECT_NEAR(r1,r1_good, eps) << sTestStr;
		}
		if (num_roots>=2) {
			EXPECT_NEAR(r2,r2_good, eps) << sTestStr;
		}
	}
}

TEST(poly_roots,solve_poly3)
{
	// `x^3+ a*x^2 + b*x + c = 0`
	// Table: 
	//     coefs (a,b,c)    num_real_roots   root1  root2
	double coefs_roots[][7] = {
		{ -6   ,11   , -6    ,   3,   1.0,  2.0, 3.0 },
		{  2   ,3    ,  4    ,   1,  -1.650629191439386,  0, 0 },
		{  0   ,-91  , -90   ,   3,  -1.0, -9.0, 10.0 }
	};

	const unsigned int nTests = sizeof(coefs_roots)/sizeof(coefs_roots[0]);

	for (unsigned int i=0;i<nTests;i++)
	{
		const double a = coefs_roots[i][0], b = coefs_roots[i][1], c = coefs_roots[i][2];
		const int num_roots_good = static_cast<int>( coefs_roots[i][3] );
		const double roots_good[3] = { coefs_roots[i][4], coefs_roots[i][5], coefs_roots[i][6] };

		double roots[3];
		int num_roots = mrpt::math::solve_poly3(roots,a,b,c);
		
		const std::string sTestStr = mrpt::format("\nSolving: x^3 + %.02f * x^2 + %.02f * x + %.02f = 0\n", a,b,c);

		EXPECT_EQ(num_roots,num_roots_good);
		for (int k=0;k<num_roots;k++) {
			bool match = false;
			for (int j=0;j<num_roots;j++)
				if (std::abs(roots[k]-roots_good[j])<eps)
					match=true;

			EXPECT_TRUE(match) << sTestStr << "k: " << k << std::endl;
		}
	}
}

TEST(poly_roots,solve_poly4)
{
	// `x^4 * a*x^3+ b*x^2 + c*x + d = 0`
	// Table: 
	//     coefs (a,b,c)    num_real_roots   roots
	double coefs_roots[][9] = {
		{-10  ,  35  , -50  ,  24    ,   4,   1.0,  2.0, 3.0, 4.0 },
		{-14  ,  35   , 50  ,   0    ,   4,  -1, 0, 5, 10 }
	};

	const unsigned int nTests = sizeof(coefs_roots)/sizeof(coefs_roots[0]);

	for (unsigned int i=0;i<nTests;i++)
	{
		const double a = coefs_roots[i][0], b = coefs_roots[i][1], c = coefs_roots[i][2], d = coefs_roots[i][3];
		const int num_roots_good = static_cast<int>( coefs_roots[i][4] );
		const double roots_good[4] = { coefs_roots[i][5], coefs_roots[i][6], coefs_roots[i][7], coefs_roots[i][8] };

		double roots[4];
		int num_roots = mrpt::math::solve_poly4(roots,a,b,c,d);
		
		const std::string sTestStr = mrpt::format("\nSolving: x^4 + %.02f * x^3 + %.02f * x^2 + %.02f * x + %.02f = 0\n", a,b,c,d);

		EXPECT_EQ(num_roots,num_roots_good);
		for (int k=0;k<num_roots;k++) {
			bool match = false;
			for (int j=0;j<num_roots;j++)
				if (std::abs(roots[k]-roots_good[j])<eps)
					match=true;

			EXPECT_TRUE(match) << sTestStr << "k: " << k << std::endl;
		}
	}
}
