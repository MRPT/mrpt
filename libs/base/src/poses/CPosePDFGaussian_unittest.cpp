/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>
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
		EXPECT_NEAR(0, (y_cov-pdf1_inv.cov).Abs().mean(), 1e-5 )
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

