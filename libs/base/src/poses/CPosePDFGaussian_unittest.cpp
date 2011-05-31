/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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

