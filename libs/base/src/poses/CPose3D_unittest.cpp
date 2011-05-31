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



class Pose3DTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }

	void test_inverse(double x1,double y1,double z1, double yaw1,double pitch1,double roll1)
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);

		const CMatrixDouble44 HM  = p1.getHomogeneousMatrixVal();
		const CMatrixDouble44 HMi = p1.getInverseHomogeneousMatrix();

		CMatrixDouble44 I4; I4.unit(4,1.0);

		EXPECT_NEAR( (HM*HMi-I4).Abs().sumAll(), 0, 1e-3 ) <<
			"HM:\n"      << HM <<
			"inv(HM):\n" << HMi <<
			"inv(HM)*HM:\n" << HM*HMi << endl;

		CPose3D p1_inv_inv = p1;

		p1_inv_inv.inverse();
		const CMatrixDouble44 HMi_from_p1_inv = p1_inv_inv.getHomogeneousMatrixVal();

		p1_inv_inv.inverse();

		EXPECT_NEAR( (p1.getAsVectorVal()-p1_inv_inv.getAsVectorVal()).Abs().sumAll(), 0, 1e-3 ) <<
			"p1: "      << p1 <<
			"p1_inv_inv: " << p1_inv_inv << endl;

		EXPECT_NEAR((HMi_from_p1_inv-HMi).Abs().sumAll(),0, 1e-4)
			<< "HMi_from_p1_inv:\n" << HMi_from_p1_inv
			<< "HMi:\n" << HMi << endl;
	}


	void test_compose(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x2,double y2,double z2, double yaw2,double pitch2,double roll2 )
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3D p2(x2,y2,z2,yaw2,pitch2,roll2);

		const CPose3D  p1_c_p2 = p1 + p2;
		const CPose3D  p1_i_p2 = p1 - p2;

		const CPose3D  p1_c_p2_i_p2 = p1_c_p2 - p1; // should be -> p2
		const CPose3D  p2_c_p1_i_p2 = p2 + p1_i_p2; // Should be -> p1

		EXPECT_NEAR(0, (p1_c_p2_i_p2.getAsVectorVal()-p2.getAsVectorVal()).Abs().sumAll(), 1e-5)
			<< "p2          : " << p2 << endl
			<< "p1_c_p2_i_p2: " << p1_c_p2_i_p2 << endl;

		EXPECT_NEAR(0, (p2_c_p1_i_p2.getAsVectorVal()-p1.getAsVectorVal()).Abs().sumAll(), 1e-5)
			<< "p1          : " << p1 << endl
			<< "p2          : " << p2 << endl
			<< "p2 matrix   : " << endl << p2.getHomogeneousMatrixVal() << endl
			<< "p1_i_p2     : " << p1_i_p2 << endl
			<< "p1_i_p2 matrix: " << endl << p1_i_p2.getHomogeneousMatrixVal() << endl
			<< "p2_c_p1_i_p2: " << p2_c_p1_i_p2 << endl;
	}

	void test_composePoint(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x,double y,double z)
	{
		const CPose3D  		p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPoint3D  	p(x,y,z);
		CPoint3D  p1_plus_p = p1 + p;

		CPoint3D  p1_plus_p2;
		p1.composePoint(p.x(),p.y(),p.z() ,p1_plus_p2.x(), p1_plus_p2.y(), p1_plus_p2.z());

		EXPECT_NEAR(0, (p1_plus_p2.getAsVectorVal()-p1_plus_p.getAsVectorVal()).Abs().sumAll(), 1e-5);

		// Inverse:
		CPoint3D  p_recov = p1_plus_p - p1;
		CPoint3D  p_recov2;
		p1.inverseComposePoint(p1_plus_p.x(),p1_plus_p.y(),p1_plus_p.z(), p_recov2.x(),p_recov2.y(),p_recov2.z() );

		EXPECT_NEAR(0, (p_recov2.getAsVectorVal()-p_recov.getAsVectorVal()).Abs().sumAll(), 1e-5);

		EXPECT_NEAR(0, (p.getAsVectorVal()-p_recov.getAsVectorVal()).Abs().sumAll(), 1e-5);
	}

	static void func_compose_point(const CArrayDouble<6+3> &x, const double &dummy, CArrayDouble<3> &Y)
	{
		CPose3D q(x[0],x[1],x[2],x[3],x[4],x[5]);
		const CPoint3D 		p(x[6+0],x[6+1],x[6+2]);
		const CPoint3D pp = q+p;
		for (int i=0;i<3;i++) Y[i]=pp[i];
	}
	static void func_inv_compose_point(const CArrayDouble<6+3> &x, const double &dummy, CArrayDouble<3> &Y)
	{
		CPose3D q(x[0],x[1],x[2],x[3],x[4],x[5]);
		const CPoint3D 		p(x[6+0],x[6+1],x[6+2]);
		const CPoint3D pp = p-q;
		Y[0]=pp.x();
		Y[1]=pp.y();
		Y[2]=pp.z();
	}

	void test_composePointJacob(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x,double y,double z,  bool use_aprox = false)
	{
		const CPose3D 	p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPoint3D 	p(x,y,z);

		CMatrixFixedNumeric<double,3,3>  df_dpoint;
		CMatrixFixedNumeric<double,3,6>  df_dpose;

		TPoint3D pp;
		p1.composePoint(x,y,z, pp.x,pp.y,pp.z, &df_dpoint, &df_dpose, NULL,  use_aprox );

		// Numerical approx:
		CMatrixFixedNumeric<double,3,3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,6> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<6+3> x_mean;
			for (int i=0;i<6;i++) x_mean[i]=p1[i];
			x_mean[6+0]=x;
			x_mean[6+1]=y;
			x_mean[6+2]=z;

			double DUMMY=0;
			CArrayDouble<6+3> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_compose_point,x_incrs, DUMMY, numJacobs );

			numJacobs.extractMatrix(0,0, num_df_dpose);
			numJacobs.extractMatrix(0,6, num_df_dpoint);
		}

		const double max_eror = use_aprox ? 0.1 : 3e-3;

		EXPECT_NEAR(0, (df_dpoint-num_df_dpoint).Abs().sumAll(), max_eror )
			<< "p1: " << p1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpoint: " << endl << num_df_dpoint << endl
			<< "Implemented method: " << endl << df_dpoint << endl
			<< "Error: " << endl << df_dpoint-num_df_dpoint << endl;

		EXPECT_NEAR(0, (df_dpose-num_df_dpose).Abs().sumAll(), max_eror )
			<< "p1: " << p1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpose: " << endl << num_df_dpose << endl
			<< "Implemented method: " << endl << df_dpose << endl
			<< "Error: " << endl << df_dpose-num_df_dpose << endl;
	}


	void test_ExpLnEqual(double x1,double y1,double z1, double yaw1,double pitch1,double roll1)
	{
		const CPose3D p1(x1,y1,z1,yaw1,pitch1,roll1);

		const CPose3D p2 = CPose3D::exp( p1.ln() );
		EXPECT_NEAR((p1.getAsVectorVal()-p2.getAsVectorVal()).Abs().sumAll(),0, 1e-6 ) << "p1: " << p1 <<endl;
	}


	void test_invComposePointJacob(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x,double y,double z)
	{
		const CPose3D 	p1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPoint3D 	p(x,y,z);

		CMatrixFixedNumeric<double,3,3>  df_dpoint;
		CMatrixFixedNumeric<double,3,6>  df_dpose;

		TPoint3D pp;
		p1.inverseComposePoint(x,y,z, pp.x,pp.y,pp.z, &df_dpoint, &df_dpose );

		// Numerical approx:
		CMatrixFixedNumeric<double,3,3> num_df_dpoint(UNINITIALIZED_MATRIX);
		CMatrixFixedNumeric<double,3,6> num_df_dpose(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<6+3> x_mean;
			for (int i=0;i<6;i++) x_mean[i]=p1[i];
			x_mean[6+0]=x;
			x_mean[6+1]=y;
			x_mean[6+2]=z;

			double DUMMY=0;
			CArrayDouble<6+3> x_incrs;
			x_incrs.assign(1e-7);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_inv_compose_point,x_incrs, DUMMY, numJacobs );

			numJacobs.extractMatrix(0,0, num_df_dpose);
			numJacobs.extractMatrix(0,6, num_df_dpoint);
		}

		EXPECT_NEAR(0, (df_dpoint-num_df_dpoint).Abs().sumAll(), 3e-3 )
			<< "p1: " << p1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpoint: " << endl << num_df_dpoint << endl
			<< "Implemented method: " << endl << df_dpoint << endl
			<< "Error: " << endl << df_dpoint-num_df_dpoint << endl;

		EXPECT_NEAR(0, (df_dpose-num_df_dpose).Abs().sumAll(), 3e-3 )
			<< "p1: " << p1 << endl
			<< "p:  " << p << endl
			<< "Numeric approximation of df_dpose: " << endl << num_df_dpose << endl
			<< "Implemented method: " << endl << df_dpose << endl
			<< "Error: " << endl << df_dpose-num_df_dpose << endl;
	}


	void test_default_values(const CPose3D &p, const std::string & label)
	{
		EXPECT_EQ(p.x(),0);
		EXPECT_EQ(p.y(),0);
		EXPECT_EQ(p.z(),0);
		EXPECT_EQ(p.yaw(),0);
		EXPECT_EQ(p.pitch(),0);
		EXPECT_EQ(p.roll(),0);
		for (size_t i=0;i<4;i++)
			for (size_t j=0;j<4;j++)
				EXPECT_NEAR(p.getHomogeneousMatrixVal()(i,j), i==j ? 1.0 : 0.0, 1e-8 )
					<< "Failed for (i,j)=" << i << "," << j << endl
					<< "Matrix is: " << endl << p.getHomogeneousMatrixVal() << endl
					<< "case was: " << label << endl;
	}

	static void func_compose_point_se3(const CArrayDouble<6> &x, const CArrayDouble<3> &P, CArrayDouble<3> &Y)
	{
		CPose3D q = CPose3D::exp(x);
		const CPoint3D 		p(P[0],P[1],P[2]);
		const CPoint3D pp = q+p;
		for (int i=0;i<3;i++) Y[i]=pp[i];
	}

	static void func_invcompose_point_se3(const CArrayDouble<6> &x, const CArrayDouble<3> &P, CArrayDouble<3> &Y)
	{
		CPose3D q = CPose3D::exp(x);
		const CPoint3D 		p(P[0],P[1],P[2]);
		const CPoint3D pp = p-q;
		for (int i=0;i<3;i++) Y[i]=pp[i];
	}


	void test_composePointJacob_se3( const CPose3D &p, const TPoint3D x_l )
	{
		CMatrixFixedNumeric<double,3,6>  df_dse3;

		TPoint3D pp;
		p.composePoint(x_l.x,x_l.y,x_l.z, pp.x,pp.y,pp.z, NULL, NULL, &df_dse3);

		// Numerical approx:
		CMatrixFixedNumeric<double,3,6> num_df_dse3(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<6> x_mean;
			for (int i=0;i<6;i++) x_mean[i]=0;

			CArrayDouble<3> P;
			for (int i=0;i<3;i++) P[i]=pp[i];

			CArrayDouble<6> x_incrs;
			x_incrs.assign(1e-9);
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_compose_point_se3,x_incrs, P, num_df_dse3 );
		}

		EXPECT_NEAR(0, (df_dse3-num_df_dse3).Abs().sumAll(), 3e-3 )
			<< "p: " << p << endl
			<< "x_l:  " << x_l << endl
			<< "Numeric approximation of df_dse3: " << endl <<num_df_dse3 << endl
			<< "Implemented method: " << endl << df_dse3 << endl
			<< "Error: " << endl << df_dse3-num_df_dse3 << endl;
	}

	void test_invComposePointJacob_se3( const CPose3D &p, const TPoint3D x_g )
	{
		CMatrixFixedNumeric<double,3,6>  df_dse3;

		TPoint3D pp;
		p.inverseComposePoint(x_g.x,x_g.y,x_g.z, pp.x,pp.y,pp.z, NULL, NULL, &df_dse3 );

		// Numerical approx:
		CMatrixFixedNumeric<double,3,6> num_df_dse3(UNINITIALIZED_MATRIX);
		{
			CArrayDouble<6> x_mean;
			for (int i=0;i<6;i++) x_mean[i]=0;

			CArrayDouble<3> P;
			for (int i=0;i<3;i++) P[i]=pp[i];

			CArrayDouble<6> x_incrs;
			x_incrs.assign(1e-9);
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_invcompose_point_se3,x_incrs, P, num_df_dse3 );
		}

		EXPECT_NEAR(0, (df_dse3-num_df_dse3).Abs().sumAll(), 3e-3 )
			<< "p: " << p << endl
			<< "x_g:  " << x_g << endl
			<< "Numeric approximation of df_dse3: " << endl <<num_df_dse3 << endl
			<< "Implemented method: " << endl << df_dse3 << endl
			<< "Error: " << endl << df_dse3-num_df_dse3 << endl;
	}

	static void func_jacob_expe_e(
		const CArrayDouble<6> &x,
		const double &dummy, CArrayDouble<12> &Y)
	{
		const CPose3D p = CPose3D::exp(x);
		//const CMatrixDouble44 R = p.getHomogeneousMatrixVal();
		p.getAs12Vector(Y);
	}

	// Check dexp(e)_de
	void check_jacob_expe_e_at_0()
	{
		CArrayDouble<6> x_mean;
		for (int i=0;i<6;i++) x_mean[i]=0;

		double dummy;
		CArrayDouble<6> x_incrs;
		x_incrs.assign(1e-9);
		CMatrixDouble numJacobs;
		mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_jacob_expe_e,x_incrs,dummy, numJacobs );

		// Theoretical matrix:
		// [ 0   -[e1]_x ]
		// [ 0   -[e2]_x ]
		// [ 0   -[e3]_x ]
		// [ I_3    0    ]
		double vals[12*6] = {
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1,
			0, 0, 0, 0,-1, 0,

			0, 0, 0, 0, 0,-1,
			0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0,

			0, 0, 0, 0, 1, 0,
			0, 0, 0,-1, 0, 0,
			0, 0, 0, 0, 0, 0,

			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0
		};
		CMatrixFixedNumeric<double,12,6>  M(vals);

		EXPECT_NEAR( (numJacobs-M).Abs().sumAll(), 0, 1e-9);
	}

	static void func_jacob_LnT_T(
		const CArrayDouble<12> &x,
		const double &dummy, CArrayDouble<6> &Y)
	{
		CPose3D p;
		p.setFrom12Vector(x);
		Y = p.ln();
	}

	// Jacobian of Ln(T) wrt T
	void check_jacob_LnT_T(double x1,double y1,double z1, double yaw1,double pitch1,double roll1)
	{
		const CPose3D p(x1,y1,z1,yaw1,pitch1,roll1);

		CMatrixFixedNumeric<double,6,12> theor_jacob;
		p.ln_jacob(theor_jacob);

		CMatrixDouble numJacobs;
		{
			CArrayDouble<12> x_mean;
			p.getAs12Vector(x_mean);

			double dummy;
			CArrayDouble<12> x_incrs;
			x_incrs.assign(1e-6);
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_jacob_LnT_T,x_incrs,dummy, numJacobs );
		}

		EXPECT_NEAR( (numJacobs-theor_jacob).Abs().sumAll(), 0, 1e-3)
			<< "Pose: " << p << endl
			<< "Pose matrix:\n" << p.getHomogeneousMatrixVal()
			<< "Num. Jacob:\n" << numJacobs << endl
			<< "Theor. Jacob:\n" << theor_jacob << endl
			<< "ERR:\n" << theor_jacob-numJacobs << endl;
	}


};

// Elemental tests:
TEST_F(Pose3DTests,DefaultValues)
{
	{
		CPose3D   p;
		test_default_values(p, "Default");
	}
	{
		CPose3D   p2;
		CPose3D   p = p2;
		test_default_values(p, "p=p2");
	}
	{
		CPose3D   p1,p2;
		test_default_values(p1+p2, "p1+p2");
		CPose3D p = p1+p2;
		test_default_values(p, "p=p1+p2");
	}
	{
		CPose3D   p1,p2;
		CPose3D p = p1-p2;
		test_default_values(p,"p1-p2");
	}
}

TEST_F(Pose3DTests,Initialization)
{
	CPose3D   p(1,2,3,0.2,0.3,0.4);
	EXPECT_NEAR(p.x(),1,  1e-7);
	EXPECT_NEAR(p.y(),2,  1e-7);
	EXPECT_NEAR(p.z(),3,  1e-7);
	EXPECT_NEAR(p.yaw(),0.2,  1e-7);
	EXPECT_NEAR(p.pitch(),0.3,  1e-7);
	EXPECT_NEAR(p.roll(),0.4,  1e-7);
}

TEST_F(Pose3DTests,OperatorBracket)
{
	CPose3D   p(1,2,3,0.2,0.3,0.4);
	EXPECT_NEAR(p[0],1,  1e-7);
	EXPECT_NEAR(p[1],2,  1e-7);
	EXPECT_NEAR(p[2],3,  1e-7);
	EXPECT_NEAR(p[3],0.2,  1e-7);
	EXPECT_NEAR(p[4],0.3,  1e-7);
	EXPECT_NEAR(p[5],0.4,  1e-7);
}

// More complex tests:
TEST_F(Pose3DTests,InverseHM)
{
	test_inverse(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
	test_inverse(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
	test_inverse(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60) );
	test_inverse(2.0,-5.0,8.0, DEG2RAD(40),DEG2RAD(-5),DEG2RAD(25) );
}

TEST_F(Pose3DTests,Compose)
{
	test_compose(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),
	             0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));
	test_compose(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),
	             4.0,5.0,6.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));

	test_compose(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),
	             2.0,-5.0,8.0, DEG2RAD(40),DEG2RAD(-5),DEG2RAD(25));

	test_compose(25.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(89),DEG2RAD(0),
	             -10.0,4.0,-8.0, DEG2RAD(20),DEG2RAD(9),DEG2RAD(0));
}

TEST_F(Pose3DTests,ComposeAndInvComposeWithPoint)
{
	test_composePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_composePoint(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DTests,ComposePointJacob)
{
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DTests,ComposePointJacobApprox)
{	// Test approximated Jacobians for very small rotations
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12   , true );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0.1),DEG2RAD(0),DEG2RAD(0),   10,11,12   , true );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0.1),DEG2RAD(0),   10,11,12   , true );
	test_composePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0.1),   10,11,12   , true );
}

TEST_F(Pose3DTests,InvComposePointJacob)
{
	test_invComposePointJacob(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   0,0,0 );
	test_invComposePointJacob(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10),   10,11,12 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60),   10.0, 20.0, 30.0 );
	test_invComposePointJacob(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40),  -5.0, -15.0, 8.0 );
}

TEST_F(Pose3DTests,ComposePointJacob_se3)
{
	test_composePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0)),   TPoint3D(10,11,12 ) );
	test_composePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0)),  TPoint3D( 10,11,12 ) );
	test_composePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0)),  TPoint3D( 10,11,12 ) );
	test_composePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10)),  TPoint3D( 10,11,12 ) );
	test_composePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60)),  TPoint3D( 10.0, 20.0, 30.0 ) );
	test_composePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40)), TPoint3D( -5.0, -15.0, 8.0 ) );
}
TEST_F(Pose3DTests,InvComposePointJacob_se3)
{
	test_invComposePointJacob_se3(CPose3D(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0)),   TPoint3D( 0,0,0  ));
	test_invComposePointJacob_se3(CPose3D(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0)),   TPoint3D( 10,11,12 ) );
	test_invComposePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0)),   TPoint3D( 10,11,12  ));
	test_invComposePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0)),   TPoint3D( 10,11,12 ) );
	test_invComposePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0)),   TPoint3D( 10,11,12 ) );
	test_invComposePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10)),   TPoint3D( 10,11,12 ) );
	test_invComposePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60)),   TPoint3D( 10.0, 20.0, 30.0 ) );
	test_invComposePointJacob_se3(CPose3D(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(-50),DEG2RAD(-40)),  TPoint3D( -5.0, -15.0, 8.0 ) );
}

TEST_F(Pose3DTests,ExpLnEqual)
{
	test_ExpLnEqual(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
	test_ExpLnEqual(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0) );
	test_ExpLnEqual(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(10),DEG2RAD(0) );
	test_ExpLnEqual(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(10) );
	test_ExpLnEqual(1.0,2.0,3.0, DEG2RAD(80),DEG2RAD(5),DEG2RAD(5) );
	test_ExpLnEqual(1.0,2.0,3.0, DEG2RAD(-20),DEG2RAD(-30),DEG2RAD(-40) );
}

TEST_F(Pose3DTests,Jacob_dExpe_de_at_0)
{
	check_jacob_expe_e_at_0();
}

TEST_F(Pose3DTests,Jacob_dLnT_dT)
{
	check_jacob_LnT_T(0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
	// JL NOTE:
	//  This function cannot be properly tested numerically, since the logm() implementation
	//  is not generic and does NOT depends on all matrix entries, thus the numerical Jacobian
	//  contains entire columns of zeros, even if the theorethical doesn't.
//	check_jacob_LnT_T(1.0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
//	check_jacob_LnT_T(1,2,3, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
//	check_jacob_LnT_T(1.0,2.0,3.0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );
//	check_jacob_LnT_T(1.0,2.0,3.0, DEG2RAD(10),DEG2RAD(20),DEG2RAD(30) );
}

