/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/math/CQuaternion.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;



class QuaternionTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }

	void test_gimbalLock(double YAW,double PITCH,double ROLL)
	{
		CQuaternionDouble  q1, q1r;
		CPose3D p1(0,0,0, YAW,PITCH,ROLL);
		p1.getAsQuaternion(q1);
		CPose3D(q1,0,0,0).getAsQuaternion(q1r);

		TPose3D t1(CPose3D(q1,0,0,0));
		TPose3D t2(CPose3D(q1r,0,0,0));

		EXPECT_NEAR(0, std::abs((CPose3D(q1,0,0,0).getAsVectorVal()-CPose3D(q1r,0,0,0).getAsVectorVal()).sum()), 1e-6);
	}

	void test_toYPRAndBack(double YAW,double PITCH,double ROLL)
	{
		CPose3D p1(0,0,0, YAW,PITCH,ROLL);
		CPose3DQuat		q1(p1);
		CPose3D			p2 = q1;

		EXPECT_NEAR(0,(p1.getRotationMatrix()-p2.getRotationMatrix()).array().abs().sum(), 1e-4) <<
			"ypr->quat->ypr failed with:" << endl
			<< "  p1:" << p1 << endl
			<< "  q1:" << q1 << endl
			<< "  p2:" << p2 << endl;

		CPose3D			p3(q1.quat(),q1[0],q1[1],q1[2] );
		EXPECT_NEAR(0,(p1.getRotationMatrix()-p3.getRotationMatrix()).array().abs().sum(), 1e-4) <<
			"pose constructor from quat failed with:" << endl
			<< "  p1:" << p1 << endl
			<< "  q1:" << q1 << endl
			<< "  p3:" << p3 << endl;
	}

	void test_lnAndExpMatches(double yaw1,double pitch1,double roll1)
	{
		const CPose3D pp(0,0,0,yaw1,pitch1,roll1);
		CQuaternionDouble q1;
		pp.getAsQuaternion(q1);

		mrpt::math::CVectorDouble q1_ln = q1.ln<mrpt::math::CVectorDouble>();
		const CQuaternionDouble q2 = CQuaternionDouble::exp(q1_ln);

		// q2 should be == q1
		EXPECT_NEAR(0, (q1-q2).array().abs().sum(), 1e-10 )
			<< "q1:\n" << q1 << endl
			<< "q2:\n" << q2 << endl
			<< "Error:\n" << (q1-q2) << endl;
	}

	void test_ExpAndLnMatches(double v0,double v1,double v2)
	{
		CArrayDouble<3> v;
		v[0]=v0;
		v[1]=v1;
		v[2]=v2;

		const CQuaternionDouble q1 = CQuaternionDouble::exp(v);
		CArrayDouble<3> q1_ln = q1.ln<CArrayDouble<3> >();

		// q1_ln should be == v
		EXPECT_NEAR(0, (q1_ln-v).array().abs().sum(), 1e-10 )
			<< "v:\n" << v << endl
			<< "q1_ln:\n" << q1_ln << endl
			<< "Error:\n" << (q1_ln-v) << endl;
	}
};

TEST_F(QuaternionTests, crossProduct)
{
	CQuaternionDouble  q1,q2,q3;

	//q1 = CQuaternionDouble(1,2,3,4); q1.normalize();
	CPose3D p1(0,0,0, DEG2RAD(10),DEG2RAD(30),DEG2RAD(-20));
	p1.getAsQuaternion(q1);

	CPose3D p2(0,0,0, DEG2RAD(30),DEG2RAD(-20),DEG2RAD(10));
	p2.getAsQuaternion(q2);

	// q3 = q1 x q2
	q3.crossProduct(q1,q2);

	const CPose3D p3 = p1 + p2;

	EXPECT_NEAR(0,std::abs((p3.getAsVectorVal()-CPose3D(q3,0,0,0).getAsVectorVal()).sum()),  1e-6) <<
		"q1 = " << q1 << endl <<
		"q1 as CPose3D = " << CPose3D(q1,0,0,0) << endl <<
		"q2 = " << q2 << endl <<
		"q2 as CPose3D = " << CPose3D(q2,0,0,0) << endl <<
		"q3 = q1 * q2 = " << q3 << endl <<
		"q3 as CPose3D = " << CPose3D(q3,0,0,0) << endl <<
		"Should be equal to p3 = p1 (+) p2 = " << p3 << endl;
}

// Use special cases: gimbal lock:
TEST_F(QuaternionTests, gimbalLock)
{
	test_gimbalLock(DEG2RAD(20),DEG2RAD(90),DEG2RAD(0));
	test_gimbalLock(DEG2RAD(20),DEG2RAD(-90),DEG2RAD(0));
}

TEST_F(QuaternionTests,ToYPRAndBack)
{
	test_toYPRAndBack(DEG2RAD(20),DEG2RAD(30),DEG2RAD(40));
	test_toYPRAndBack(DEG2RAD(20),DEG2RAD(30),DEG2RAD(40));
	test_toYPRAndBack(DEG2RAD(30),DEG2RAD(90),DEG2RAD(0));
	test_toYPRAndBack(DEG2RAD(-30),DEG2RAD(90),DEG2RAD(0));
	test_toYPRAndBack(DEG2RAD(-30),DEG2RAD(88),DEG2RAD(60));
	test_toYPRAndBack(DEG2RAD(-30),DEG2RAD(10),DEG2RAD(60));

}

TEST_F(QuaternionTests,LnAndExpMatches)
{
	const double list_test_YPR_angles_degrees[][3] =  {
		{  0, 0, 0 },
		{ -1, 0, 0 }, {  1, 0, 0 },
		{  0,-1, 0 }, {  0, 1, 0 },
		{  0, 0, -1}, {  0, 0,  1},
		{  40, 0, 0 }, {  0, 40, 0 }, {  0, 0, 40 },
		{ -40, 0, 0 }, {  0,-40, 0 }, {  0, 0,-40 },
		{ -30, 20, 50 }, { -30, 20,-50 }, {  30,-20,-50 }
	};

	const size_t N = sizeof(list_test_YPR_angles_degrees)/sizeof(list_test_YPR_angles_degrees[0]);
	for (size_t i=0;i<N;i++)
		test_lnAndExpMatches(DEG2RAD(list_test_YPR_angles_degrees[i][0]),DEG2RAD(list_test_YPR_angles_degrees[i][1]),DEG2RAD(list_test_YPR_angles_degrees[i][2]));
}

TEST_F(QuaternionTests,ExpAndLnMatches)
{
	const double list_test_XYZ[][3] =  {
		{  0, 0, 0 },
		{ -1, 0, 0 }, {  1, 0, 0 },
		{  0,-1, 0 }, {  0, 1, 0 },
		{  0, 0, -1}, {  0, 0,  1},
		{  1e-3, 0, 0}, {  0, 1e-3, 0}, {0,0, 1e-3},
		{ -1e-3, 0, 0}, {  0,-1e-3, 0}, {0,0,-1e-3},
		{ -0.1,0.2,0.3 },{-0.1,-0.2,0.3 },{-0.1,-0.2,-0.3 },{-0.1,0.2,-0.3 },{0.1,0.2,-0.3 },{0.1,0.2,0.3}
	};

	const size_t N = sizeof(list_test_XYZ)/sizeof(list_test_XYZ[0]);
	for (size_t i=0;i<N;i++)
		test_ExpAndLnMatches(list_test_XYZ[i][0],list_test_XYZ[i][1],list_test_XYZ[i][2]);
}


