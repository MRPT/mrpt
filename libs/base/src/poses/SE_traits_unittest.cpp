/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/slam.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


template <class POSE_TYPE>
class SE_traits_tests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}
	virtual void TearDown() {  }

	typedef  mrpt::poses::SE_traits<POSE_TYPE::rotation_dimensions> SE_TYPE;

	struct TParams
	{
		typename SE_TYPE::pose_t  P1, D, P2;
	};

	static void func_numeric(
		const CArrayDouble<2*SE_TYPE::VECTOR_SIZE> &x, 
		const TParams &params, 
		CArrayDouble<SE_TYPE::VECTOR_SIZE> &Y)
	{
		const CArrayNumeric<double,6> eps1 = x.slice<0,6>();
		const CArrayNumeric<double,6> eps2 = x.slice<6,6>();

		const CPose3D 	P1 = CPose3D::exp(eps1) + params.P1;
		const CPose3D 	P2 = CPose3D::exp(eps2) + params.P2;
		const CPose3D 	&Pd = params.D;

		const CPose3D   P1DP2inv(P1.getHomogeneousMatrixVal() * Pd.getHomogeneousMatrixVal() * P2.getInverseHomogeneousMatrix());
		// Pseudo-logarithm: 
		for (int i=0;i<3;i++)	// X Y Z
			Y[i] = P1DP2inv[i];

		const CArrayDouble<3> ln_rot = P1DP2inv.ln_rotation();
		for (int i=0;i<3;i++)
			Y[3+i] = ln_rot[i];
	}

	void test_jacobs_P1DP2inv(
		double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
		double xd,double yd,double zd, double yawd,double pitchd,double rolld,
		double x2,double y2,double z2, double yaw2,double pitch2,double roll2)
	{
		const CPose3D 	P1(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3D 	Pd(xd,yd,zd,yawd,pitchd,rolld);
		const CPose3D 	P2(x2,y2,z2,yaw2,pitch2,roll2);

		const CPose3D   P1DP2inv(P1.getHomogeneousMatrixVal() * Pd.getHomogeneousMatrixVal() * P2.getInverseHomogeneousMatrix());

		// Theoretical results:
		CMatrixFixedNumeric<double,6,6>  J1,J2;
		SE_TYPE::jacobian_dP1DP2inv_depsilon(P1DP2inv, &J1, &J2);

		// Numerical approx:
		CMatrixFixedNumeric<double,6,6> num_J1,num_J2;
		{
			CArrayDouble<6+6> x_mean;
			for (int i=0;i<6+6;i++) x_mean[i]=0;

			TParams params;
			params.P1 = P1;
			params.D  = Pd;
			params.P2 = P2;

			CArrayDouble<6+6> x_incrs;
			x_incrs.assign(1e-4);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_numeric,x_incrs, params, numJacobs );

			numJacobs.extractMatrix(0,0, num_J1);
			numJacobs.extractMatrix(0,6, num_J2);
		}

		const double max_eror = 1e-3;

		EXPECT_NEAR(0, (num_J1-J1).Abs().sumAll(), max_eror )
			<< "p1: " << P1 << endl
			<< "d: "  << Pd << endl
			<< "p2: " << P2 << endl
			<< "Numeric J1:\n" << num_J1 << endl
			<< "Implemented J1:\n" << J1 << endl
			<< "Error:\n" << J1-num_J1 << endl;

		EXPECT_NEAR(0, (num_J2-J2).Abs().sumAll(), max_eror )
			<< "p1: " << P1 << endl
			<< "d: "  << Pd << endl
			<< "p2: " << P2 << endl
			<< "Numeric J2:\n" << num_J2 << endl
			<< "Implemented J2:\n" << J2 << endl
			<< "Error:\n" << J2-num_J2 << endl;

	}
};

typedef SE_traits_tests<mrpt::poses::CPose3D> SE3_traits_tests;
//typedef SE_traits_tests<mrpt::poses::CPose2D> SE2_traits_tests;

TEST_F(SE3_traits_tests,SE3_jacobs)
{
	test_jacobs_P1DP2inv(
		0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),
		0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),
		0,0,0, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );

	test_jacobs_P1DP2inv(
		0,0,0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),
		0,0,0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),
		0,0,0, DEG2RAD(20),DEG2RAD(0),DEG2RAD(0) );

	test_jacobs_P1DP2inv(
		0,0,0, DEG2RAD(10),DEG2RAD(0),DEG2RAD(0),
		0,0,0, DEG2RAD(12),DEG2RAD(0),DEG2RAD(0),
		0,0,0, DEG2RAD(20),DEG2RAD(0),DEG2RAD(0) );

	test_jacobs_P1DP2inv(
		0,0,0, DEG2RAD(10),DEG2RAD(20),DEG2RAD(30),
		0,0,0, DEG2RAD(4),DEG2RAD(3),DEG2RAD(8),
		0,0,0, DEG2RAD(15),DEG2RAD(25),DEG2RAD(35) );

	test_jacobs_P1DP2inv(
		10,20,30, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),
		-5,-5,-5, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),
		5,15,25, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );

	test_jacobs_P1DP2inv(
		10,20,30, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),
		-5.2,-5.3,-4.9, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0),
		5,15,25, DEG2RAD(0),DEG2RAD(0),DEG2RAD(0) );

	test_jacobs_P1DP2inv(
		10,20,30, DEG2RAD(10),DEG2RAD(20),DEG2RAD(30),
		4.7,4.8,5.3, DEG2RAD(4),DEG2RAD(3),DEG2RAD(8),
		15,25,35, DEG2RAD(15),DEG2RAD(25),DEG2RAD(35) );

}
