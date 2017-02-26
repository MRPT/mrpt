/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/random.h>
#include <mrpt/math/transform_gaussian.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;



class PosePDFGaussTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }

	static CPosePDFGaussian  generateRandomPose2DPDF(double x,double y, double phi, double std_scale)
	{
		CMatrixDouble31  r;
		mrpt::random::randomGenerator.drawGaussian1DMatrix(r, 0,std_scale);
		CMatrixDouble33 cov;
		cov.multiply_AAt(r);  // random semi-definite positive matrix:
		for (int i=0;i<3;i++) cov(i,i)+=1e-7;
		CPosePDFGaussian pdf( CPose2D(x,y,phi), cov );
		return pdf;
	}

	static void func_inverse(const CArrayDouble<3> &x, const double &dummy, CArrayDouble<3> &Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		const CPose2D p1(x[0],x[1],x[2]);
		const CPose2D p1_inv = CPose2D() - p1;
		for (int i=0;i<3;i++) Y[i]=p1_inv[i];
	}

	void testPoseInverse(
		double x,double y, double phi, double std_scale)
	{
		CPosePDFGaussian  pdf1 = generateRandomPose2DPDF(x,y,phi, std_scale);

		CPosePDFGaussian  pdf1_inv;
		pdf1.inverse(pdf1_inv);

		// Numeric approximation:
		CArrayDouble<3> y_mean;
		CMatrixFixedNumeric<double,3,3>  y_cov;
		{
			CArrayDouble<3> x_mean;
			for (int i=0;i<3;i++) x_mean[i]=pdf1.mean[i];

			CMatrixFixedNumeric<double,3,3>  x_cov =pdf1.cov;

			double DUMMY=0;
			CArrayDouble<3> x_incrs;
			x_incrs.assign(1e-6);
			transform_gaussian_linear(x_mean,x_cov,func_inverse,DUMMY, y_mean,y_cov, x_incrs );
		}
		// Compare:
		EXPECT_NEAR(0, (y_cov-pdf1_inv.cov).array().abs().mean(), 1e-5 )
			<< "pdf1 mean: " << pdf1.mean << endl
			<< "Numeric approximation of covariance: " << endl << y_cov << endl
			<< "Returned covariance: " << endl << pdf1_inv.cov << endl;
	}


};

TEST_F(PosePDFGaussTests,Inverse)
{
	testPoseInverse(0,0,0, 0.01);
	testPoseInverse(0,0,0, 0.1);

	testPoseInverse(1,0,0, 0.1);
	testPoseInverse(0,1,0, 0.1);
	testPoseInverse(0,0,1, 0.1);

	testPoseInverse(-5,0,0, 0.1);
	testPoseInverse(0,-5,0, 0.1);
	testPoseInverse(0,0,-5, 0.1);

	testPoseInverse(4,6,DEG2RAD(10), 0.1);
	testPoseInverse(4,6,DEG2RAD(-10), 0.1);

	testPoseInverse(-7,2,DEG2RAD(30), 0.1);
	testPoseInverse(-7,2,DEG2RAD(-30), 0.1);
}

