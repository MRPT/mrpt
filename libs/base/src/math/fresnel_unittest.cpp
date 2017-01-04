/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/math/fresnel.h>
#include <mrpt/system/os.h>
#include <gtest/gtest.h>
#include <cmath>

TEST(fresnel,fresnelc)
{
	// fresnelc()
	const double test_values[][2] = {
		{ 0.0,   .0},
		{ 0.79788456080286541, 0.721705924292605 },
		{ 1.0, 0.779893400376823 },
		{ 0.4, 0.397480759172359 },
		{ 1.5, 0.445261176039822 },
		{ 2.0, 0.488253406075341 },
		{ 2.4, 0.554961405856428 },
		{ 3.34, 0.407099627096608 },
		{ 50.0, 0.499999189430728 },
		{ -0.4, -0.397480759172359 },
		{ -1.5, -0.445261176039822 },
		{ -2.0, -0.488253406075341 },
		{ -2.4, -0.554961405856428 },
		{ -3.34, -0.407099627096608 },
		{ -50.0, -0.499999189430728 }
	};

	const unsigned int nTests = sizeof(test_values)/sizeof(test_values[0]);

	for (unsigned int i=0;i<nTests;i++)
	{
		const double x = test_values[i][0], val_good = test_values[i][1];
		const double val = mrpt::math::fresnel_cos_integral(x);
		EXPECT_NEAR(val, val_good, 1e-5) << " x: " << x << "\n val_good: " << val_good << "\n val: " << val << "\n";
	}
}

TEST(fresnel, fresnels)
{
	// fresnelc()
	const double test_values[][2] = {
		{ 0.0,   .0 },
		{ 1.0, 0.438259147390355 },
		{ 1.5, 0.697504960082093 },
		{ 2.4, 0.619689964945684 },
		{ 50.0, 0.493633802585939 },
		{ -2.0, -0.343415678363698 },
		{ -2.4, -0.619689964945684 },
		{ -3.34, -0.479600423968308 },
		{ -50.0, -0.493633802585939 }
	};

	const unsigned int nTests = sizeof(test_values) / sizeof(test_values[0]);

	for (unsigned int i = 0; i<nTests; i++)
	{
		const double x = test_values[i][0], val_good = test_values[i][1];
		const double val = mrpt::math::fresnel_sin_integral(x);
		EXPECT_NEAR(val, val_good, 1e-5) << " x: " << x << "\n val_good: " << val_good << "\n val: " << val << "\n";
	}
}
