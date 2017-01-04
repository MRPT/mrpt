/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/jacobians.h>
#include <mrpt/random.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;



class Pose3DQuatTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }

	void test_compose(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x2,double y2,double z2, double yaw2,double pitch2,double roll2 )
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3D p2(x2,y2,z2,yaw2,pitch2,roll2);

		const CPose3D  p1_c_p2 = p1 + p2;
		const CPose3D  p1_i_p2 = p1 - p2;


		CPose3DQuat  q1=CPose3DQuat(p1);
		CPose3DQuat  q2=CPose3DQuat(p2);

		CPose3DQuat  q1_c_q2=q1+q2;
		CPose3DQuat  q1_i_q2=q1-q2;

		CPose3D p_q1_c_q2 = CPose3D(q1_c_q2);
		CPose3D p_q1_i_q2 = CPose3D(q1_i_q2);

		EXPECT_NEAR(0, (p1_c_p2.getAsVectorVal()-p_q1_c_q2.getAsVectorVal()).array().abs().sum(), 1e-5) <<
			"p1_c_p2: " << p1_c_p2 << endl <<
			"q1_c_p2: " << p_q1_c_q2 << endl;

		EXPECT_NEAR(0, (p1_i_p2.getAsVectorVal()-p_q1_i_q2.getAsVectorVal()).array().abs().sum(), 1e-5) <<
			"p1_i_p2: " << p1_i_p2 << endl <<
			"q1_i_p2: " << p_q1_i_q2 << endl;

		// Test + operator: trg new var
		{
			CPose3DQuat C = q1;
			CPose3DQuat A = C + q2;
			EXPECT_NEAR(0, (A.getAsVectorVal()-q1_c_q2.getAsVectorVal()).array().abs().sum(), 1e-6);
		}
		// Test + operator: trg same var
		{
			CPose3DQuat A = q1;
			A = A + q2;
			EXPECT_NEAR(0, (A.getAsVectorVal()-q1_c_q2.getAsVectorVal()).array().abs().sum(), 1e-6);
		}
		// Test =+ operator
		{
			CPose3DQuat A = q1;
			A += q2;
			EXPECT_NEAR(0, (A.getAsVectorVal()-q1_c_q2.getAsVectorVal()).array().abs().sum(), 1e-6);
		}

	}

	void test_composePoint(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x,double y,double z)
	{
		const CPose3D  		p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3DQuat  	q1(p1);
		const CPoint3D  	p(x,y,z);

		CPoint3D  p1_plus_p = p1 + p;
		CPoint3D  q1_plus_p = q1 + p;

		EXPECT_NEAR(0, (p1_plus_p.getAsVectorVal()-q1_plus_p.getAsVectorVal()).array().abs().sum(), 1e-5) <<
			"p1: " << p1 << endl <<
			"q1: " << q1 << endl <<
			"p: " << p << endl <<
			"p1_plus_p: " << p1_plus_p << endl <<
			"q1_plus_p: " << q1_plus_p << endl;

	}




	static void func_compose_point(const CArrayDouble<7+3> &x, const double &dummy, CArrayDouble<3> &Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3DQuat 	q(x[0],x[1],x[2],CQuaternionDouble(x[3],x[4],x[5],x[6]));
		q.quat().normalize();
		const CPoint3D 		p(x[7+0],x[7+1],x[7+2]);
		const CPoint3D pp = q+p;
		for (int i=0;i<3;i++) Y[i]=pp[i];
	}

	void test_composePointJacob(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x,double y,double z)
	{
		const CPose3DQuat  	q1( CPose3D(x1,y1,z1,yaw1,pitch1,roll1) );
		const CPoint3D  	p(x,y,z);

		CMatrixFixedNumeric<double,3,3> df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,7> df_dpose(UNINITIALIZED_MATRIX);

		TPoint3D l;
		q1.composePoint(x,y,z, l.x,l.y,l.z, &df_dpoint, &df_dpose);

		// Numerical approximation:
		CMatrixFixedNumeric<double,3,3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,7> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<7+3> x_mean;
			for (int i=0;i<7;i++) x_mean[i]=q1[i];
			x_mean[7+0]=x;
			x_mean[7+1]=y;
			x_mean[7+2]=z;

			double DUMMY=0;
			CArrayDouble<7+3> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_compose_point,x_incrs, DUMMY, numJacobs );

			numJacobs.extractMatrix(0,0, num_df_dpose);
			numJacobs.extractMatrix(0,7, num_df_dpoint);
		}


		// Compare:
		EXPECT_NEAR(0, (df_dpoint-num_df_dpoint).array().abs().sum(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpoint: " << endl << num_df_dpoint << endl
			<< "Implemented method: " << endl << df_dpoint << endl
			<< "Error: " << endl << df_dpoint-num_df_dpoint << endl;

		EXPECT_NEAR(0, (df_dpose-num_df_dpose).array().abs().sum(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpose: " << endl << num_df_dpose << endl
			<< "Implemented method: " << endl << df_dpose << endl
			<< "Error: " << endl << df_dpose-num_df_dpose << endl;

	}

	void test_invComposePoint(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x,double y,double z)
	{
		const CPose3D		p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3DQuat  	q1(p1);
		const CPoint3D  	p(x,y,z);

		CPoint3D  p_minus_p1 = p - p1;
		CPoint3D  p_minus_q1 = p - q1;

		CPoint3D  p_rec = q1 + p_minus_q1;


		EXPECT_NEAR(0, (p_minus_p1.getAsVectorVal()-p_minus_q1.getAsVectorVal()).array().abs().sum(), 1e-5) <<
			"p_minus_p1: " << p_minus_p1 << endl <<
			"p_minus_q1: " << p_minus_q1 << endl;

		EXPECT_NEAR(0, (p_rec.getAsVectorVal()-p.getAsVectorVal()).array().abs().sum(), 1e-5) <<
			"p_rec: " << p_rec << endl <<
			"p: " << p << endl;
	}

	static void func_inv_compose_point(const CArrayDouble<7+3> &x, const double &dummy, CArrayDouble<3> &Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3DQuat 	q(x[0],x[1],x[2],CQuaternionDouble(x[3],x[4],x[5],x[6]));
		q.quat().normalize();
		const CPoint3D 		p(x[7+0],x[7+1],x[7+2]);
		const CPoint3D pp = p-q;
		Y[0]=pp.x();
		Y[1]=pp.y();
		Y[2]=pp.z();
	}

	void test_invComposePointJacob(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x,double y,double z)
	{
		const CPose3DQuat  	q1( CPose3D(x1,y1,z1,yaw1,pitch1,roll1) );
		const CPoint3D  	p(x,y,z);

		CMatrixFixedNumeric<double,3,3> df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,7> df_dpose(UNINITIALIZED_MATRIX);

		TPoint3D l;
		q1.inverseComposePoint(x,y,z, l.x,l.y,l.z, &df_dpoint, &df_dpose);

		// Also check the returned point, not just the jacobian:
		TPoint3D theorical;
		{
			const double qr = q1.quat().r(); const double qx = q1.quat().x();  const double qy = q1.quat().y(); const double qz = q1.quat().z();
			const double Ax = x-x1; // ax  -  x;
			const double Ay = y-y1; // ay  -  y;
			const double Az = z-z1; // az  -  z;
			theorical.x = Ax + 2*(Ay)*(qr*qz + qx*qy) - 2*(Az)*(qr*qy - qx*qz) - 2*(square(qy) + square(qz))*(Ax);
			theorical.y = Ay - 2*(Ax)*(qr*qz - qx*qy) + 2*(Az)*(qr*qx + qy*qz) - 2*(square(qx) + square(qz))*(Ay);
			theorical.z = Az + 2*(Ax)*(qr*qy + qx*qz) - 2*(Ay)*(qr*qx - qy*qz) - 2*(square(qx) + square(qy))*(Az);
		}
		EXPECT_NEAR(theorical.x,l.x, 1e-5);
		EXPECT_NEAR(theorical.y,l.y, 1e-5);
		EXPECT_NEAR(theorical.z,l.z, 1e-5);

		// Numerical approximation:
		CMatrixFixedNumeric<double,3,3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,7> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<7+3> x_mean;
			for (int i=0;i<7;i++) x_mean[i]=q1[i];
			x_mean[7+0]=x;
			x_mean[7+1]=y;
			x_mean[7+2]=z;

			double DUMMY=0;
			CArrayDouble<7+3> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_inv_compose_point,x_incrs, DUMMY, numJacobs );

			numJacobs.extractMatrix(0,0, num_df_dpose);
			numJacobs.extractMatrix(0,7, num_df_dpoint);
		}

		// Compare:
		EXPECT_NEAR(0, (df_dpoint-num_df_dpoint).array().abs().sum(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "from pose: " << CPose3D(x1,y1,z1,yaw1,pitch1,roll1) << endl
			<< "p:  " << p << endl
			<< "local:  " << l << endl
			<< "Numeric approximation of df_dpoint: " << endl << num_df_dpoint << endl
			<< "Implemented method: " << endl << df_dpoint << endl
			<< "Error: " << endl << df_dpoint-num_df_dpoint << endl;

		EXPECT_NEAR(0, (df_dpose-num_df_dpose).array().abs().sum(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "from pose: " << CPose3D(x1,y1,z1,yaw1,pitch1,roll1) << endl
			<< "p:  " << p << endl
			<< "local:  " << l << endl
			<< "Numeric approximation of df_dpose: " << endl << num_df_dpose << endl
			<< "Implemented method: " << endl << df_dpose << endl
			<< "Error: " << endl << df_dpose-num_df_dpose << endl;
	}


	void test_fromYPRAndBack(double x1,double y1,double z1, double yaw1,double pitch1,double roll1)
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3DQuat q1(p1);
		const CPose3D p1r = q1;

		EXPECT_NEAR(0, (p1.getHomogeneousMatrixVal()-q1.getHomogeneousMatrixVal()).array().abs().sum(), 1e-5) <<
			"p1.getHomogeneousMatrixVal():\n" << p1.getHomogeneousMatrixVal() << endl <<
			"q1.getHomogeneousMatrixVal():\n" << q1.getHomogeneousMatrixVal() << endl;

		EXPECT_NEAR(0, (p1.getAsVectorVal()-p1r.getAsVectorVal()).array().abs().sum(), 1e-5) <<
			"p1: " << p1 << endl <<
			"q1: " << q1 << endl <<
			"p1r: " << p1r << endl;
	}

	void test_unaryInverse(double x1,double y1,double z1, double yaw1,double pitch1,double roll1)
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3D p1_inv = -p1;

		const CPose3DQuat q1(p1);
		const CPose3DQuat q1_inv = -q1;

		EXPECT_NEAR(0, (p1_inv.getHomogeneousMatrixVal()-q1_inv.getHomogeneousMatrixVal()).array().abs().sum(), 1e-5) <<
			"p1_inv.getHomogeneousMatrixVal():\n" << p1_inv.getHomogeneousMatrixVal() << endl <<
			"q1_inv.getHomogeneousMatrixVal():\n" << q1_inv.getHomogeneousMatrixVal() << endl;

	}

	void test_copy(double x1,double y1,double z1, double yaw1,double pitch1,double roll1)
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);

		const CPose3DQuat q1(p1);
		const CPose3DQuat q2 = q1;

		EXPECT_NEAR(0, (q1.getHomogeneousMatrixVal()-q2.getHomogeneousMatrixVal()).array().abs().sum(), 1e-5) <<
			"q1.getHomogeneousMatrixVal():\n" << q1.getHomogeneousMatrixVal() << endl <<
			"q2.getHomogeneousMatrixVal():\n" << q2.getHomogeneousMatrixVal() << endl;

	}

	void test_composeAndInvComposePoint(
		double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
		double x, double y, double z)
	{
		const CPose3DQuat q1(CPose3D(x1,y1,z1,yaw1,pitch1,roll1));
		TPoint3D pp(x,y,z), aux;
		q1.composePoint( x,y,z, pp.x,pp.y,pp.z);
		q1.inverseComposePoint( pp.x,pp.y,pp.z, aux.x, aux.y, aux.z );

		EXPECT_NEAR(x,aux.x, 1e-7);
		EXPECT_NEAR(y,aux.y, 1e-7);
		EXPECT_NEAR(z,aux.z, 1e-7);
	}

	void test_composePoint_vs_CPose3D(
		double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
		double x, double y, double z)
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3DQuat q1(p1);
		TPoint3D pt1,pt2;
		p1.composePoint( x,y,z, pt1.x,pt1.y,pt1.z);
		q1.composePoint( x,y,z, pt2.x,pt2.y,pt2.z);

		EXPECT_NEAR(pt1.x,pt2.x, 1e-7);
		EXPECT_NEAR(pt1.y,pt2.y, 1e-7);
		EXPECT_NEAR(pt1.z,pt2.z, 1e-7);
	}

	void test_invComposePoint_vs_CPose3D(
		double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
		double x, double y, double z)
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3DQuat q1(p1);
		TPoint3D pt1,pt2;
		p1.inverseComposePoint( x,y,z, pt1.x,pt1.y,pt1.z);
		q1.inverseComposePoint( x,y,z, pt2.x,pt2.y,pt2.z);

		EXPECT_NEAR(pt1.x,pt2.x, 1e-7);
		EXPECT_NEAR(pt1.y,pt2.y, 1e-7);
		EXPECT_NEAR(pt1.z,pt2.z, 1e-7);

		{
			CPose3DQuat q = mrpt::poses::CPose3DQuat(p1);

			float gx=x,gy=y,gz=z;

			double dist,yaw,pitch;
			p1.sphericalCoordinates(TPoint3D(gx,gy,gz), dist,yaw,pitch);

			double lx,ly,lz;
			p1.inverseComposePoint(gx,gy,gz, lx,ly,lz);

			double lx2,ly2,lz2;
			q.inverseComposePoint(gx,gy,gz, lx2,ly2,lz2);

			EXPECT_NEAR(lx,lx2, 1e-7);
			EXPECT_NEAR(ly,ly2, 1e-7);
			EXPECT_NEAR(lz,lz2, 1e-7);
		}

	}
	
	static void func_spherical_coords(const CArrayDouble<7+3> &x, const double &dummy, CArrayDouble<3> &Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CPose3DQuat 	q(x[0],x[1],x[2],CQuaternionDouble(x[3],x[4],x[5],x[6]));
		q.quat().normalize();
		const TPoint3D 		p(x[7+0],x[7+1],x[7+2]);
		q.sphericalCoordinates(p,Y[0],Y[1],Y[2]);
	}

	void test_sphericalCoords(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x,double y,double z)
	{
		const CPose3DQuat  	q1( CPose3D(x1,y1,z1,yaw1,pitch1,roll1) );
		const TPoint3D  	p(x,y,z);

		CMatrixFixedNumeric<double,3,3> df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,7> df_dpose(UNINITIALIZED_MATRIX);

		double hr,hy,hp;
		q1.sphericalCoordinates(p,hr,hy,hp, &df_dpoint, &df_dpose);

		// Numerical approximation:
		CMatrixFixedNumeric<double,3,3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,7> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<7+3> x_mean;
			for (int i=0;i<7;i++) x_mean[i]=q1[i];
			x_mean[7+0]=x;
			x_mean[7+1]=y;
			x_mean[7+2]=z;

			double DUMMY=0;
			CArrayDouble<7+3> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_spherical_coords,x_incrs, DUMMY, numJacobs );

			numJacobs.extractMatrix(0,0, num_df_dpose);
			numJacobs.extractMatrix(0,7, num_df_dpoint);
		}


		// Compare:
		EXPECT_NEAR(0, (df_dpoint-num_df_dpoint).array().abs().sum(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpoint: " << endl << num_df_dpoint << endl
			<< "Implemented method: " << endl << df_dpoint << endl
			<< "Error: " << endl << df_dpoint-num_df_dpoint << endl;

		EXPECT_NEAR(0, (df_dpose-num_df_dpose).array().abs().sum(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpose: " << endl << num_df_dpose << endl
			<< "Implemented method: " << endl << df_dpose << endl
			<< "Error: " << endl << df_dpose-num_df_dpose << endl;

	}

	static void func_normalizeJacob(const CArrayDouble<4> &x, const double &dummy, CArrayDouble<4> &Y)
	{
		MRPT_UNUSED_PARAM(dummy);
		CQuaternionDouble q;
		for (int i=0;i<4;i++) q[i]=x[i];
		q.normalize();
		for (int i=0;i<4;i++) Y[i]=q[i];
	}

	void test_normalizeJacob(double yaw1,double pitch1,double roll1)
	{
		const CPose3D pp(0,0,0,yaw1,pitch1,roll1);
		CQuaternionDouble q1;
		pp.getAsQuaternion(q1);

		CMatrixFixedNumeric<double,4,4> df_dpose(UNINITIALIZED_MATRIX);
		q1.normalizationJacobian(df_dpose);


		// Numerical approximation:
		CMatrixFixedNumeric<double,4,4> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<4> x_mean;
			for (int i=0;i<4;i++) x_mean[i]=q1[i];

			double DUMMY=0;
			CArrayDouble<4> x_incrs;
			x_incrs.assign(1e-5);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_normalizeJacob,x_incrs, DUMMY, numJacobs );

			numJacobs.extractMatrix(0,0, num_df_dpose);
		}


		// Compare:
		EXPECT_NEAR(0, (df_dpose-num_df_dpose).array().abs().sum(), 3e-3 )
			<< "q1: " << q1 << endl
			<< "Numeric approximation of df_dpose: " << endl << num_df_dpose << endl
			<< "Implemented method: " << endl << df_dpose << endl
			<< "Error: " << endl << df_dpose-num_df_dpose << endl;
	}

};


TEST_F(Pose3DQuatTests,FromYPRAndBack)
{
	test_fromYPRAndBack(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
	test_fromYPRAndBack(1.0,2.0,3.0, DEG2RAD(90),DEG2RAD(0),DEG2RAD(0));
	test_fromYPRAndBack(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60));
	test_fromYPRAndBack(1.0,2.0,3.0, DEG2RAD(179),DEG2RAD(0),DEG2RAD(60));
	test_fromYPRAndBack(1.0,2.0,3.0, DEG2RAD(-179),DEG2RAD(0),DEG2RAD(60));
	test_fromYPRAndBack(1.0,2.0,3.0, DEG2RAD(30),DEG2RAD( 89),DEG2RAD(0));
	test_fromYPRAndBack(1.0,2.0,3.0, DEG2RAD(30),DEG2RAD(-89),DEG2RAD(0));
}

TEST_F(Pose3DQuatTests,UnaryInverse)
{
	test_unaryInverse(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
	test_unaryInverse(1.0,2.0,3.0, DEG2RAD(90),DEG2RAD(0),DEG2RAD(0));
	test_unaryInverse(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60));
	test_unaryInverse(1.0,2.0,3.0, DEG2RAD(179),DEG2RAD(0),DEG2RAD(60));
	test_unaryInverse(1.0,2.0,3.0, DEG2RAD(-179),DEG2RAD(0),DEG2RAD(60));
	test_unaryInverse(1.0,2.0,3.0, DEG2RAD(30),DEG2RAD( 89),DEG2RAD(0));
	test_unaryInverse(1.0,2.0,3.0, DEG2RAD(30),DEG2RAD(-89),DEG2RAD(0));
}


TEST_F(Pose3DQuatTests,CopyOperator)
{
	test_copy(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
	test_copy(1.0,2.0,3.0, DEG2RAD(90),DEG2RAD(0),DEG2RAD(0));
	test_copy(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60));
	test_copy(1.0,2.0,3.0, DEG2RAD(30),DEG2RAD(-89),DEG2RAD(0));
}


TEST_F(Pose3DQuatTests,Compose)
{
	test_compose(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),
	             2.0,-5.0,8.0, DEG2RAD(40),DEG2RAD(-5),DEG2RAD(25));

	test_compose(25.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(90),DEG2RAD(0),
	             -10.0,4.0,-8.0, DEG2RAD(20),DEG2RAD(9),DEG2RAD(0));
}

TEST_F(Pose3DQuatTests,ComposeWithPoint)
{
	test_composePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DQuatTests,ComposeWithPointJacob)
{
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DQuatTests,InvComposeWithPoint)
{
	test_invComposePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePoint(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_invComposePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_invComposePoint(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_invComposePoint(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DQuatTests,InvComposeWithPointJacob)
{
	test_invComposePointJacob(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   0,0,0 );
	test_invComposePointJacob(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   1,2,3 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   0,0,0);
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DQuatTests,ComposeInvComposePoint)
{
	test_composeAndInvComposePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composeAndInvComposePoint(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composeAndInvComposePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_composeAndInvComposePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_composeAndInvComposePoint(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_composeAndInvComposePoint(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DQuatTests,ComposePoint_vs_CPose3D)
{
	test_composePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_composePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_composePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_composePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DQuatTests,InvComposePoint_vs_CPose3D)
{
	test_invComposePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_invComposePoint_vs_CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );

	for (size_t i=0;i<10;i++)
	{
		std::vector<double> v(9);
		mrpt::random::randomGenerator.drawGaussian1DVector(v,0,1);
		test_invComposePoint_vs_CPose3D(v[0],v[1],v[2],v[3],v[4],v[5],  v[6],v[7],v[8]);
	}
}

TEST_F(Pose3DQuatTests,SphericalCoordsJacobian)
{
	test_sphericalCoords(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_sphericalCoords(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_sphericalCoords(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_sphericalCoords(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_sphericalCoords(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_sphericalCoords(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DQuatTests,NormalizationJacobian)
{
	test_normalizeJacob(DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
	test_normalizeJacob(DEG2RAD(10),DEG2RAD(0),DEG2RAD(0));
	test_normalizeJacob(DEG2RAD(0),DEG2RAD(10),DEG2RAD(0));
	test_normalizeJacob(DEG2RAD(0),DEG2RAD(0),DEG2RAD(10));
	test_normalizeJacob(DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60));
	test_normalizeJacob(DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40));
}

