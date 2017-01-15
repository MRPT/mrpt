/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/SE_traits.h>
#include <mrpt/math/jacobians.h>
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
		typename SE_TYPE::array_t  eps1, eps2;
		for (int i=0;i<SE_TYPE::VECTOR_SIZE;i++)
		{
			eps1[i] = x[0+i];
			eps2[i] = x[SE_TYPE::VECTOR_SIZE+i];
		}

		POSE_TYPE incr1, incr2;
		SE_TYPE::exp(eps1,incr1);
		SE_TYPE::exp(eps2,incr2);

		const POSE_TYPE 	P1 = incr1 + params.P1;
		const POSE_TYPE 	P2 = incr2 + params.P2;
		const POSE_TYPE 	&Pd = params.D;

		const CPose3D   P1DP2inv_(CMatrixDouble44(P1.getHomogeneousMatrixVal() * Pd.getHomogeneousMatrixVal() * P2.getInverseHomogeneousMatrix()));
		const POSE_TYPE P1DP2inv(P1DP2inv_);

		// Pseudo-logarithm:
		SE_TYPE::pseudo_ln(P1DP2inv,Y);
	}

	void test_jacobs_P1DP2inv(
		double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
		double xd,double yd,double zd, double yawd,double pitchd,double rolld,
		double x2,double y2,double z2, double yaw2,double pitch2,double roll2)
	{
		const CPose3D 	P1_(x1,y1,z1,yaw1,pitch1,roll1);
		const CPose3D 	Pd_(xd,yd,zd,yawd,pitchd,rolld);
		const CPose3D 	P2_(x2,y2,z2,yaw2,pitch2,roll2);

		const POSE_TYPE   P1(P1_);
		const POSE_TYPE   Pd(Pd_);
		const POSE_TYPE   P2(P2_);

		const CPose3D   P1DP2inv_( CMatrixDouble44(P1.getHomogeneousMatrixVal() * Pd.getHomogeneousMatrixVal() * P2.getInverseHomogeneousMatrix()));
		const POSE_TYPE P1DP2inv(P1DP2inv_); // Convert to 2D if needed

		static const int DIMS = SE_TYPE::VECTOR_SIZE;

		// Theoretical results:
		CMatrixFixedNumeric<double,DIMS,DIMS>  J1,J2;
		SE_TYPE::jacobian_dP1DP2inv_depsilon(P1DP2inv, &J1, &J2);

		// Numerical approx:
		CMatrixFixedNumeric<double,DIMS,DIMS> num_J1,num_J2;
		{
			CArrayDouble<2*DIMS> x_mean;
			for (int i=0;i<DIMS+DIMS;i++) x_mean[i]=0;

			TParams params;
			params.P1 = P1;
			params.D  = Pd;
			params.P2 = P2;

			CArrayDouble<DIMS+DIMS> x_incrs;
			x_incrs.assign(1e-4);
			CMatrixDouble numJacobs;
			mrpt::math::jacobians::jacob_numeric_estimate(x_mean,func_numeric,x_incrs, params, numJacobs );

			numJacobs.extractMatrix(0,0, num_J1);
			numJacobs.extractMatrix(0,DIMS, num_J2);
		}

		const double max_eror = 1e-3;

		EXPECT_NEAR(0, (num_J1-J1).array().abs().sum(), max_eror )
			<< "p1: " << P1 << endl
			<< "d: "  << Pd << endl
			<< "p2: " << P2 << endl
			<< "Numeric J1:\n" << num_J1 << endl
			<< "Implemented J1:\n" << J1 << endl
			<< "Error:\n" << J1-num_J1 << endl;

		EXPECT_NEAR(0, (num_J2-J2).array().abs().sum(), max_eror )
			<< "p1: " << P1 << endl
			<< "d: "  << Pd << endl
			<< "p2: " << P2 << endl
			<< "Numeric J2:\n" << num_J2 << endl
			<< "Implemented J2:\n" << J2 << endl
			<< "Error:\n" << J2-num_J2 << endl;
	}


	void do_all_jacobs_test()
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
};

typedef SE_traits_tests<mrpt::poses::CPose3D> SE3_traits_tests;
typedef SE_traits_tests<mrpt::poses::CPose2D> SE2_traits_tests;

TEST_F(SE3_traits_tests,SE3_jacobs)
{
	do_all_jacobs_test();
}

TEST_F(SE2_traits_tests,SE2_jacobs)
{
	do_all_jacobs_test();
}
