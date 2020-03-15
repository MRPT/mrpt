/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/math/transform_gaussian.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>
#include <Eigen/Dense>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

template class mrpt::CTraitsTest<CPosePDFGaussian>;

class PosePDFGaussTests : public ::testing::Test
{
   protected:
	void SetUp() override {}
	void TearDown() override {}
	static CPosePDFGaussian generateRandomPose2DPDF(
		double x, double y, double phi, double std_scale)
	{
		CMatrixDouble31 r;
		mrpt::random::getRandomGenerator().drawGaussian1DMatrix(
			r, 0, std_scale);
		CMatrixDouble33 cov;
		cov.matProductOf_AAt(r);  // random semi-definite positive matrix:
		for (int i = 0; i < 3; i++) cov(i, i) += 1e-7;
		CPosePDFGaussian pdf(CPose2D(x, y, phi), cov);
		return pdf;
	}

	static void func_inverse(
		const CVectorFixedDouble<3>& x, const double& dummy,
		CVectorFixedDouble<3>& Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		const CPose2D p1(x[0], x[1], x[2]);
		const CPose2D p1_inv = CPose2D() - p1;
		for (int i = 0; i < 3; i++) Y[i] = p1_inv[i];
	}

	void testPoseInverse(double x, double y, double phi, double std_scale)
	{
		CPosePDFGaussian pdf1 = generateRandomPose2DPDF(x, y, phi, std_scale);

		CPosePDFGaussian pdf1_inv;
		pdf1.inverse(pdf1_inv);

		// Numeric approximation:
		CVectorFixedDouble<3> y_mean;
		CMatrixFixed<double, 3, 3> y_cov;
		{
			CVectorFixedDouble<3> x_mean;
			for (int i = 0; i < 3; i++) x_mean[i] = pdf1.mean[i];

			CMatrixFixed<double, 3, 3> x_cov = pdf1.cov;

			double DUMMY = 0;
			CVectorFixedDouble<3> x_incrs;
			x_incrs.fill(1e-6);
			transform_gaussian_linear(
				x_mean, x_cov, func_inverse, DUMMY, y_mean, y_cov, x_incrs);
		}
		// Compare:
		EXPECT_NEAR(0, (y_cov - pdf1_inv.cov).array().abs().mean(), 1e-5)
			<< "pdf1 mean: " << pdf1.mean << endl
			<< "Numeric approximation of covariance: " << endl
			<< y_cov << endl
			<< "Returned covariance: " << endl
			<< pdf1_inv.cov << endl;
	}
};

TEST_F(PosePDFGaussTests, Inverse)
{
	testPoseInverse(0, 0, 0, 0.01);
	testPoseInverse(0, 0, 0, 0.1);

	testPoseInverse(1, 0, 0, 0.1);
	testPoseInverse(0, 1, 0, 0.1);
	testPoseInverse(0, 0, 1, 0.1);

	testPoseInverse(-5, 0, 0, 0.1);
	testPoseInverse(0, -5, 0, 0.1);
	testPoseInverse(0, 0, -5, 0.1);

	testPoseInverse(4, 6, 10.0_deg, 0.1);
	testPoseInverse(4, 6, -10.0_deg, 0.1);

	testPoseInverse(-7, 2, 30.0_deg, 0.1);
	testPoseInverse(-7, 2, -30.0_deg, 0.1);
}
