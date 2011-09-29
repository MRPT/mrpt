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



class Pose3DQuatPDFGaussTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }

	static CPose3DQuatPDFGaussian  generateRandomPoseQuat3DPDF(double x,double y, double z, double yaw, double pitch, double roll, double std_scale)
	{
		return CPose3DQuatPDFGaussian(generateRandomPose3DPDF(x,y,z,yaw,pitch,roll,std_scale));
	}

	static CPose3DPDFGaussian  generateRandomPose3DPDF(double x,double y, double z, double yaw, double pitch, double roll, double std_scale)
	{
		CMatrixDouble61  r;
		mrpt::random::randomGenerator.drawGaussian1DMatrix(r, 0,std_scale);
		CMatrixDouble66 cov;
		cov.multiply_AAt(r);  // random semi-definite positive matrix:
		for (int i=0;i<6;i++) cov(i,i)+=1e-7;
		CPose3DPDFGaussian  p6pdf( CPose3D(x,y,z,yaw,pitch,roll), cov );
		return p6pdf;
	}


	void test_toFromYPRGauss(double yaw, double pitch,double roll)
	{
		// Random pose:
		CPose3DPDFGaussian  p1ypr = generateRandomPose3DPDF(1.0,2.0,3.0, yaw,pitch,roll, 0.1);
		CPose3DQuatPDFGaussian  p1quat = CPose3DQuatPDFGaussian(p1ypr);

		// Convert back to a 6x6 representation:
		CPose3DPDFGaussian  p2ypr = CPose3DPDFGaussian(p1quat);

		EXPECT_NEAR(0,(p2ypr.cov - p1ypr.cov).Abs().mean(), 1e-2)
			<< "p1ypr: " << endl << p1ypr << endl
			<< "p1quat : " << endl << p1quat << endl
			<< "p2ypr : " << endl << p2ypr << endl;
	}

	static void func_compose(const CArrayDouble<2*7> &x, const double &dummy, CArrayDouble<7> &Y)
	{
		const CPose3DQuat p1(x[0],x[1],x[2],CQuaternionDouble(x[3],x[4],x[5],x[6]));
		const CPose3DQuat p2(x[7+0],x[7+1],x[7+2],CQuaternionDouble(x[7+3],x[7+4],x[7+5],x[7+6]));
		const CPose3DQuat p = p1+p2;
		for (int i=0;i<7;i++) Y[i]=p[i];
	}
	static void func_inv_compose(const CArrayDouble<2*7> &x, const double &dummy, CArrayDouble<7> &Y)
	{
		const CPose3DQuat p1(x[0],x[1],x[2],CQuaternionDouble(x[3],x[4],x[5],x[6]));
		const CPose3DQuat p2(x[7+0],x[7+1],x[7+2],CQuaternionDouble(x[7+3],x[7+4],x[7+5],x[7+6]));
		const CPose3DQuat p = p1-p2;
		for (int i=0;i<7;i++) Y[i]=p[i];
	}

	void testPoseComposition(
		double x,double y, double z, double yaw, double pitch, double roll, double std_scale,
		double x2,double y2, double z2, double yaw2, double pitch2, double roll2, double std_scale2 )
	{
		CPose3DQuatPDFGaussian  p7pdf1 = generateRandomPoseQuat3DPDF(x,y,z,yaw,pitch,roll, std_scale);
		CPose3DQuatPDFGaussian  p7pdf2 = generateRandomPoseQuat3DPDF(x2,y2,z2,yaw2,pitch2,roll2, std_scale2);

		CPose3DQuatPDFGaussian  p7_comp = p7pdf1 + p7pdf2;

		// Numeric approximation:
		CArrayDouble<7> y_mean;
		CMatrixFixedNumeric<double,7,7>  y_cov;
		{
			CArrayDouble<2*7> x_mean;
			for (int i=0;i<7;i++) x_mean[i]=p7pdf1.mean[i];
			for (int i=0;i<7;i++) x_mean[7+i]=p7pdf2.mean[i];

			CMatrixFixedNumeric<double,14,14>  x_cov;
			x_cov.insertMatrix(0,0, p7pdf1.cov);
			x_cov.insertMatrix(7,7, p7pdf2.cov);

			double DUMMY=0;
			CArrayDouble<2*7> x_incrs;
			x_incrs.assign(1e-6);
			transform_gaussian_linear(x_mean,x_cov,func_compose,DUMMY, y_mean,y_cov, x_incrs );

		}
		// Compare:
		EXPECT_NEAR(0, (y_cov-p7_comp.cov).Abs().mean(), 1e-2 )
			<< "p1 mean: " << p7pdf1.mean << endl
			<< "p2 mean: " << p7pdf2.mean << endl
			<< "Numeric approximation of covariance: " << endl << y_cov << endl
			<< "Returned covariance: " << endl << p7_comp.cov << endl;

	}

	static void func_inverse(const CArrayDouble<7> &x, const double &dummy, CArrayDouble<7> &Y)
	{
		const CPose3DQuat p1(x[0],x[1],x[2],CQuaternionDouble(x[3],x[4],x[5],x[6]));
		const CPose3DQuat p1_inv ( -p1 );
		for (int i=0;i<7;i++) Y[i]=p1_inv[i];
	}

	void testCompositionJacobian(
		double x,double y, double z, double yaw, double pitch, double roll,
		double x2,double y2, double z2, double yaw2, double pitch2, double roll2)
	{
		const CPose3DQuat  q1( CPose3D(x,y,z,yaw,pitch,roll) );
		const CPose3DQuat  q2( CPose3D(x2,y2,z2,yaw2,pitch2,roll2) );

		// Theoretical Jacobians:
		CMatrixDouble77  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);
		CPose3DQuatPDF::jacobiansPoseComposition(
			q1,  // x
			q2,     // u
			df_dx,
			df_du );

		// Numerical approximation:
		CMatrixDouble77  num_df_dx(UNINITIALIZED_MATRIX), num_df_du(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<2*7> x_mean;
			for (int i=0;i<7;i++) x_mean[i]=q1[i];
			for (int i=0;i<7;i++) x_mean[7+i]=q2[i];

			double DUMMY=0;
			CArrayDouble<2*7> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_compose,x_incrs, DUMMY, numJacobs );

			numJacobs.extractMatrix(0,0, num_df_dx);
			numJacobs.extractMatrix(0,7, num_df_du);
		}

		// Compare:
		EXPECT_NEAR(0, (df_dx-num_df_dx).Abs().sumAll(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "q2: " << q2 << endl
			<< "Numeric approximation of df_dx: " << endl << num_df_dx << endl
			<< "Implemented method: " << endl << df_dx << endl
			<< "Error: " << endl << df_dx-num_df_dx << endl;

		EXPECT_NEAR(0, (df_du-num_df_du).Abs().sumAll(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "q2: " << q2 << endl
			<< "Numeric approximation of df_du: " << endl << num_df_du << endl
			<< "Implemented method: " << endl << df_du << endl
			<< "Error: " << endl << df_du-num_df_du << endl;
	}

	void testInverse(double x,double y, double z, double yaw, double pitch, double roll, double std_scale)
	{
		CPose3DQuatPDFGaussian  p7pdf1 = generateRandomPoseQuat3DPDF(x,y,z,yaw,pitch,roll, std_scale);

		CPose3DQuatPDFGaussian  p7_inv = -p7pdf1;

		// Numeric approximation:
		CArrayDouble<7> y_mean;
		CMatrixFixedNumeric<double,7,7>  y_cov;
		{
			CArrayDouble<7> x_mean;
			for (int i=0;i<7;i++) x_mean[i]=p7pdf1.mean[i];

			CMatrixFixedNumeric<double,7,7>  x_cov;
			x_cov.insertMatrix(0,0, p7pdf1.cov);

			double DUMMY=0;
			CArrayDouble<7> x_incrs;
			x_incrs.assign(1e-6);
			transform_gaussian_linear(x_mean,x_cov,func_inverse,DUMMY, y_mean,y_cov, x_incrs );
		}

		// Compare:
		EXPECT_NEAR(0, (y_cov-p7_inv.cov).Abs().mean(), 1e-2 )
			<< "p1 mean: " << p7pdf1.mean << endl
			<< "inv mean: " << p7_inv.mean << endl
			<< "Numeric approximation of covariance: " << endl << y_cov << endl
			<< "Returned covariance: " << endl << p7_inv.cov << endl
			<< "Error: " << endl << y_cov-p7_inv.cov << endl;
	}


	void testPoseInverseComposition(
		double x,double y, double z, double yaw, double pitch, double roll, double std_scale,
		double x2,double y2, double z2, double yaw2, double pitch2, double roll2, double std_scale2 )
	{
		CPose3DQuatPDFGaussian  p7pdf1 = generateRandomPoseQuat3DPDF(x,y,z,yaw,pitch,roll, std_scale);
		CPose3DQuatPDFGaussian  p7pdf2 = generateRandomPoseQuat3DPDF(x2,y2,z2,yaw2,pitch2,roll2, std_scale2);

		CPose3DQuatPDFGaussian  p7_comp = p7pdf1 - p7pdf2;

		// Numeric approximation:
		CArrayDouble<7> y_mean;
		CMatrixFixedNumeric<double,7,7>  y_cov;
		{
			CArrayDouble<2*7> x_mean;
			for (int i=0;i<7;i++) x_mean[i]=p7pdf1.mean[i];
			for (int i=0;i<7;i++) x_mean[7+i]=p7pdf2.mean[i];

			CMatrixFixedNumeric<double,14,14>  x_cov;
			x_cov.insertMatrix(0,0, p7pdf1.cov);
			x_cov.insertMatrix(7,7, p7pdf2.cov);

			double DUMMY=0;
			CArrayDouble<2*7> x_incrs;
			x_incrs.assign(1e-6);
			transform_gaussian_linear(x_mean,x_cov,func_inv_compose,DUMMY, y_mean,y_cov, x_incrs );

		}
		// Compare:
		EXPECT_NEAR(0, (y_cov-p7_comp.cov).Abs().mean(), 1e-2 )
			<< "p1 mean: " << p7pdf1.mean << endl
			<< "p2 mean: " << p7pdf2.mean << endl
			<< "Numeric approximation of covariance: " << endl << y_cov << endl
			<< "Returned covariance: " << endl << p7_comp.cov << endl;

	}


};

/* TODO: Make tests for
  - changeCoordinatesReference
*/


TEST_F(Pose3DQuatPDFGaussTests,ToYPRGaussPDFAndBack)
{
	test_toFromYPRGauss(DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60));
	test_toFromYPRGauss(DEG2RAD(30),DEG2RAD(88),DEG2RAD(0));
	test_toFromYPRGauss(DEG2RAD(30),DEG2RAD(89.5),DEG2RAD(0));
	// The formulas break at pitch=90, but this we cannot avoid...
}

TEST_F(Pose3DQuatPDFGaussTests,CompositionJacobian)
{
	testCompositionJacobian(0,0,0,DEG2RAD(2),DEG2RAD(0),DEG2RAD(0),  0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
	testCompositionJacobian(1,2,3,DEG2RAD(2),DEG2RAD(0),DEG2RAD(0),  -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
	testCompositionJacobian(1,-2,3,DEG2RAD(2),DEG2RAD(0),DEG2RAD(0),  -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
	testCompositionJacobian(1,2,-3,DEG2RAD(2),DEG2RAD(0),DEG2RAD(0),  -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
	testCompositionJacobian(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(70), -8,45,10,DEG2RAD(50),DEG2RAD(-10),DEG2RAD(30) );
	testCompositionJacobian(1,2,3,DEG2RAD(20),DEG2RAD(-80),DEG2RAD(70), -8,45,10,DEG2RAD(50),DEG2RAD(-10),DEG2RAD(30) );
	testCompositionJacobian(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(-70), -8,45,10,DEG2RAD(50),DEG2RAD(-10),DEG2RAD(30) );
	testCompositionJacobian(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(70), -8,45,10,DEG2RAD(-50),DEG2RAD(-10),DEG2RAD(30) );
	testCompositionJacobian(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(70), -8,45,10,DEG2RAD(50),DEG2RAD(10),DEG2RAD(30) );
	testCompositionJacobian(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(70), -8,45,10,DEG2RAD(50),DEG2RAD(-10),DEG2RAD(-30) );
}

TEST_F(Pose3DQuatPDFGaussTests,Inverse)
{
	testInverse(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(0,0,0,DEG2RAD(10),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(0,0,0,DEG2RAD(0),DEG2RAD(10),DEG2RAD(0), 0.1 );
	testInverse(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(10), 0.1 );

	testInverse(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.2 );

	testInverse(1,2,3,DEG2RAD(30),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(-1,2,3,DEG2RAD(30),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(1,2,-3,DEG2RAD(30),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(-1,2,-3,DEG2RAD(30),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(1,2,3,DEG2RAD(-30),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(-1,2,3,DEG2RAD(-30),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(1,2,-3,DEG2RAD(-30),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(-1,2,-3,DEG2RAD(-30),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testInverse(1,2,3,DEG2RAD(0),DEG2RAD(30),DEG2RAD(0), 0.1 );
	testInverse(-1,2,3,DEG2RAD(0),DEG2RAD(30),DEG2RAD(0), 0.1 );
	testInverse(1,2,-3,DEG2RAD(0),DEG2RAD(30),DEG2RAD(0), 0.1 );
	testInverse(-1,2,-3,DEG2RAD(0),DEG2RAD(30),DEG2RAD(0), 0.1 );
	testInverse(1,2,3,DEG2RAD(0),DEG2RAD(-30),DEG2RAD(0), 0.1 );
	testInverse(-1,2,3,DEG2RAD(0),DEG2RAD(-30),DEG2RAD(0), 0.1 );
	testInverse(1,2,-3,DEG2RAD(0),DEG2RAD(-30),DEG2RAD(0), 0.1 );
	testInverse(-1,2,-3,DEG2RAD(0),DEG2RAD(-30),DEG2RAD(0), 0.1 );
	testInverse(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(30), 0.1 );
	testInverse(-1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(30), 0.1 );
	testInverse(1,2,-3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(30), 0.1 );
	testInverse(-1,2,-3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(30), 0.1 );
	testInverse(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(-30), 0.1 );
	testInverse(-1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(-30), 0.1 );
	testInverse(1,2,-3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(-30), 0.1 );
	testInverse(-1,2,-3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(-30), 0.1 );
}

TEST_F(Pose3DQuatPDFGaussTests,Composition)
{
	testPoseComposition(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1,  0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1,  -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );

	testPoseComposition(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(70), 0.1, -8,45,10,DEG2RAD(50),DEG2RAD(-10),DEG2RAD(30), 0.1 );
	testPoseComposition(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(70), 0.2, -8,45,10,DEG2RAD(50),DEG2RAD(-10),DEG2RAD(30), 0.2 );

	testPoseComposition(1,2,3,DEG2RAD(10),DEG2RAD(0),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseComposition(1,2,3,DEG2RAD(0),DEG2RAD(10),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(10), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(10),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(10),DEG2RAD(0), 0.1 );
	testPoseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(10), 0.1 );
}

TEST_F(Pose3DQuatPDFGaussTests,InverseComposition)
{
	testPoseInverseComposition(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1,  0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseInverseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1,  -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );

	testPoseInverseComposition(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(70), 0.1, -8,45,10,DEG2RAD(50),DEG2RAD(-10),DEG2RAD(30), 0.1 );
	testPoseInverseComposition(1,2,3,DEG2RAD(20),DEG2RAD(80),DEG2RAD(70), 0.2, -8,45,10,DEG2RAD(50),DEG2RAD(-10),DEG2RAD(30), 0.2 );

	testPoseInverseComposition(1,2,3,DEG2RAD(10),DEG2RAD(0),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseInverseComposition(1,2,3,DEG2RAD(0),DEG2RAD(10),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseInverseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(10), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseInverseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(10),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testPoseInverseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(10),DEG2RAD(0), 0.1 );
	testPoseInverseComposition(1,2,3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1, -8,45,10,DEG2RAD(0),DEG2RAD(0),DEG2RAD(10), 0.1 );
}
