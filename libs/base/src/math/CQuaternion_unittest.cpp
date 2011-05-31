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

		EXPECT_NEAR(0, std::abs((CPose3D(q1,0,0,0).getAsVectorVal()-CPose3D(q1r,0,0,0).getAsVectorVal()).sumAll()), 1e-6);
	}

	void test_toYPRAndBack(double YAW,double PITCH,double ROLL)
	{
		CPose3D p1(0,0,0, YAW,PITCH,ROLL);
		CPose3DQuat		q1(p1);
		CPose3D			p2 = q1;

		EXPECT_NEAR(0,(p1.getRotationMatrix()-p2.getRotationMatrix()).Abs().sumAll(), 1e-4) <<
			"ypr->quat->ypr failed with:" << endl
			<< "  p1:" << p1 << endl
			<< "  q1:" << q1 << endl
			<< "  p2:" << p2 << endl;

		CPose3D			p3(q1.quat(),q1[0],q1[1],q1[2] );
		EXPECT_NEAR(0,(p1.getRotationMatrix()-p3.getRotationMatrix()).Abs().sumAll(), 1e-4) <<
			"pose constructor from quat failed with:" << endl
			<< "  p1:" << p1 << endl
			<< "  q1:" << q1 << endl
			<< "  p3:" << p3 << endl;
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

	EXPECT_NEAR(0,std::abs((p3.getAsVectorVal()-CPose3D(q3,0,0,0).getAsVectorVal()).sumAll()),  1e-6) <<
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

