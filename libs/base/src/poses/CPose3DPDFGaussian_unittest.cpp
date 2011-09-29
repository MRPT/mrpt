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



class Pose3DPDFGaussTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }


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


	void testToQuatPDFAndBack(double x,double y, double z, double yaw, double pitch, double roll, double std_scale)
	{
		CPose3DPDFGaussian  p6pdf = generateRandomPose3DPDF(x,y,z,yaw,pitch,roll, std_scale);
		//cout << "p6pdf: " << p6pdf << endl;
		CPose3DQuatPDFGaussian  p7pdf = CPose3DQuatPDFGaussian(p6pdf);
		//cout << "p7pdf: " << p7pdf << endl;
		CPose3DPDFGaussian  p6pdf_recov = CPose3DPDFGaussian(p7pdf);
		//cout << "p6pdf_recov: " << p6pdf_recov  << endl;

		const double val_mean_error = (p6pdf_recov.mean.getAsVectorVal() - p6pdf.mean.getAsVectorVal()).Abs().mean();
		const double cov_mean_error = (p6pdf_recov.cov - p6pdf.cov).Abs().mean();
		//cout << "cov err: " << cov_mean_error << " " << "val_mean_error: " << val_mean_error << endl;
		EXPECT_TRUE(val_mean_error < 1e-8);
		EXPECT_TRUE(cov_mean_error < 1e-8);
	}


	static void func_compose(const CArrayDouble<12> &x, const double &dummy, CArrayDouble<6> &Y)
	{
		const CPose3D p1(x[0],x[1],x[2],x[3],x[4],x[5]);
		const CPose3D p2(x[6+0],x[6+1],x[6+2],x[6+3],x[6+4],x[6+5]);
		const CPose3D p = p1+p2;
		for (int i=0;i<6;i++) Y[i]=p[i];
	}
	static void func_inv_compose(const CArrayDouble<2*6> &x, const double &dummy, CArrayDouble<6> &Y)
	{
		const CPose3D p1(x[0],x[1],x[2],x[3],x[4],x[5]);
		const CPose3D p2(x[6+0],x[6+1],x[6+2],x[6+3],x[6+4],x[6+5]);
		const CPose3D p = p1-p2;
		for (int i=0;i<6;i++) Y[i]=p[i];
	}

	void testPoseComposition(
		double x,double y, double z, double yaw, double pitch, double roll, double std_scale,
		double x2,double y2, double z2, double yaw2, double pitch2, double roll2, double std_scale2 )
	{
		CPose3DPDFGaussian  p6pdf1 = generateRandomPose3DPDF(x,y,z,yaw,pitch,roll, std_scale);
		CPose3DPDFGaussian  p6pdf2 = generateRandomPose3DPDF(x2,y2,z2,yaw2,pitch2,roll2, std_scale2);

		// With "+"/"-" operators:
		{
			CPose3DPDFGaussian  p6_comp = p6pdf1 + p6pdf2;

			// Numeric approximation:
			CArrayDouble<6> y_mean;
			CMatrixFixedNumeric<double,6,6>  y_cov;
			{
				CArrayDouble<12> x_mean;
				for (int i=0;i<6;i++) x_mean[i]=p6pdf1.mean[i];
				for (int i=0;i<6;i++) x_mean[6+i]=p6pdf2.mean[i];

				CMatrixFixedNumeric<double,12,12>  x_cov;
				x_cov.insertMatrix(0,0, p6pdf1.cov);
				x_cov.insertMatrix(6,6, p6pdf2.cov);

				double DUMMY=0;
				CArrayDouble<12> x_incrs;
				x_incrs.assign(1e-6);
				transform_gaussian_linear(x_mean,x_cov,func_compose,DUMMY, y_mean,y_cov, x_incrs );

			}
			// Compare:
			EXPECT_NEAR(0, (y_cov-p6_comp.cov).Abs().mean(), 1e-2 )
				<< "Numeric approximation of covariance: " << endl << y_cov << endl
				<< "Returned covariance: " << endl << p6_comp.cov << endl;

			// Try to recover the individual poses:

			CPose3DPDFGaussian  p2_rec = p6_comp - p6pdf1;
//			cout << "p2 rec: " << p2_rec << endl;
			CPose3DPDFGaussian  p1_rec = p6_comp - p6pdf2;
//			cout << "p1 rec: " << p1_rec << endl;
		}
	}


	void testCompositionJacobian(
		double x,double y, double z, double yaw, double pitch, double roll,
		double x2,double y2, double z2, double yaw2, double pitch2, double roll2)
	{
		const CPose3D  q1(x,y,z,yaw,pitch,roll);
		const CPose3D  q2(x2,y2,z2,yaw2,pitch2,roll2);

		// Theoretical Jacobians:
		CMatrixDouble66  df_dx(UNINITIALIZED_MATRIX), df_du(UNINITIALIZED_MATRIX);
		CPose3DPDF::jacobiansPoseComposition(
			q1,  // x
			q2,     // u
			df_dx,
			df_du );

		// Numerical approximation:
		CMatrixDouble66  num_df_dx(UNINITIALIZED_MATRIX), num_df_du(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<2*6> x_mean;
			for (int i=0;i<6;i++) x_mean[i]=q1[i];
			for (int i=0;i<6;i++) x_mean[6+i]=q2[i];

			double DUMMY=0;
			CArrayDouble<2*6> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_compose,x_incrs, DUMMY, numJacobs );

			numJacobs.extractMatrix(0,0, num_df_dx);
			numJacobs.extractMatrix(0,6, num_df_du);
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

	void testPoseInverseComposition(
		double x,double y, double z, double yaw, double pitch, double roll, double std_scale,
		double x2,double y2, double z2, double yaw2, double pitch2, double roll2, double std_scale2 )
	{
		CPose3DPDFGaussian  p6pdf1 = generateRandomPose3DPDF(x,y,z,yaw,pitch,roll, std_scale);
		CPose3DPDFGaussian  p6pdf2 = generateRandomPose3DPDF(x2,y2,z2,yaw2,pitch2,roll2, std_scale2);

		CPose3DPDFGaussian  p6_comp = p6pdf1 - p6pdf2;

		// Numeric approximation:
		CArrayDouble<6> y_mean;
		CMatrixFixedNumeric<double,6,6>  y_cov;
		{
			CArrayDouble<2*6> x_mean;
			for (int i=0;i<6;i++) x_mean[i]=p6pdf1.mean[i];
			for (int i=0;i<6;i++) x_mean[6+i]=p6pdf2.mean[i];

			CMatrixFixedNumeric<double,12,12>  x_cov;
			x_cov.insertMatrix(0,0, p6pdf1.cov);
			x_cov.insertMatrix(6,6, p6pdf2.cov);

			double DUMMY=0;
			CArrayDouble<2*6> x_incrs;
			x_incrs.assign(1e-6);
			transform_gaussian_linear(x_mean,x_cov,func_inv_compose,DUMMY, y_mean,y_cov, x_incrs );
		}
		// Compare:
		EXPECT_NEAR(0, (y_cov-p6_comp.cov).Abs().mean(), 1e-2 )
			<< "p1 mean: " << p6pdf1.mean << endl
			<< "p2 mean: " << p6pdf2.mean << endl
			<< "Numeric approximation of covariance: " << endl << y_cov << endl
			<< "Returned covariance: " << endl << p6_comp.cov << endl;
	}



};

/* TODO: Make tests for
  - operators: +=, -=, +, -
  - changeCoordinatesReference
  - inverse
*/


TEST_F(Pose3DPDFGaussTests,ToQuatGaussPDFAndBack)
{
	testToQuatPDFAndBack(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testToQuatPDFAndBack(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.2 );

	testToQuatPDFAndBack(6,-2,-3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.1 );
	testToQuatPDFAndBack(6,-2,-3,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0), 0.2 );

	testToQuatPDFAndBack(6,-2,-3,DEG2RAD(10),DEG2RAD(40),DEG2RAD(5), 0.1 );
	testToQuatPDFAndBack(6,-2,-3,DEG2RAD(10),DEG2RAD(40),DEG2RAD(5), 0.2 );

	testToQuatPDFAndBack(6,-2,-3,DEG2RAD(-50),DEG2RAD(87),DEG2RAD(20), 0.1 );
	testToQuatPDFAndBack(6,-2,-3,DEG2RAD(-50),DEG2RAD(87),DEG2RAD(20), 0.2 );

	testToQuatPDFAndBack(6,-2,-3,DEG2RAD(-50),DEG2RAD(-87),DEG2RAD(20), 0.1 );
	testToQuatPDFAndBack(6,-2,-3,DEG2RAD(-50),DEG2RAD(-87),DEG2RAD(20), 0.2 );
}

TEST_F(Pose3DPDFGaussTests,CompositionJacobian)
{
	testCompositionJacobian(0,0,0,DEG2RAD(2),DEG2RAD(0),DEG2RAD(0),  0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
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

TEST_F(Pose3DPDFGaussTests,InverseComposition)
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
