/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPose3DRotVec.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

class Pose3DRotVecTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }

	void test_conversions()
	{
	    // CPose3DRotVec -> CPose3D -> CPose3DRotVec
	    CPose3DRotVec poseRVT(0.2,0.3,0.4,1,2,3);
	    CPose3D pose3D(poseRVT);
	    CPose3DRotVec newPoseRVT_1(pose3D);

	    EXPECT_NEAR(0, (poseRVT.getAsVectorVal()-newPoseRVT_1.getAsVectorVal()).array().abs().sum(), 1e-5)
            << "EULER: " << endl
            << "pRVT        : " << poseRVT << endl
			<< "newPoseRVT  : " << newPoseRVT_1 << endl;

        // CPose3DRotVec -> CPose3DQuat -> CPose3DRotVec
        CPose3DQuat poseQuat(poseRVT);
        CPose3DRotVec newPoseRVT_2(poseQuat);

	    EXPECT_NEAR(0, (poseRVT.getAsVectorVal()-newPoseRVT_2.getAsVectorVal()).array().abs().sum(), 1e-5)
            << "Quat: " << endl
            << "pRVT        : " << poseRVT << endl
			<< "newPoseRVT  : " << newPoseRVT_2 << endl;

	}

	void test_default_values(const CPose3DRotVec &p, const std::string & label)
	{
		MRPT_UNUSED_PARAM(label);
		EXPECT_EQ(p.x(),0);
		EXPECT_EQ(p.y(),0);
		EXPECT_EQ(p.z(),0);
		EXPECT_EQ(p.rx(),0);
		EXPECT_EQ(p.ry(),0);
		EXPECT_EQ(p.rz(),0);

#if 0
		CMatrixDouble44 HM = p.getHomogeneousMatrixVal();
		for (size_t i=0;i<4;i++)
			for (size_t j=0;j<4;j++)
				EXPECT_NEAR(HM(i,j), i==j ? 1.0 : 0.0, 1e-8 )
					<< "Failed for (i,j)=" << i << "," << j << endl
					<< "Matrix is: " << endl << HM << endl
					<< "case was: " << label << endl;
#endif
	}

	void test_compose(double x1,double y1,double z1, double yaw1,double pitch1,double roll1,
	                 double x2,double y2,double z2, double yaw2,double pitch2,double roll2)
	{
        CPose3DRotVec p1;
	    p1.setFromXYZAndAngles(x1,y1,z1,yaw1,pitch1,roll1);
//	    double md = p1.m_rotvec.norm();
//		p1.m_rotvec /= md;
//		p1.m_rotvec *= 0.009;

		CPose3DRotVec p2;
		p2.setFromXYZAndAngles(x2,y2,z2,yaw2,pitch2,roll2);
//		md = p2.m_rotvec.norm();
//		p2.m_rotvec /= md;
//		p2.m_rotvec *= 0.009;
//		cout << "p1: " << p1.m_rotvec << endl;
//		cout << "p2: " << p2.m_rotvec << endl;

//        cout << "Compose: " << endl;
        const CPose3DRotVec  p1_c_p2 = p1 + p2;
		const CPose3DRotVec  p1_i_p2 = p1 - p2;

		const CPose3DRotVec  p1_c_p2_i_p2 = p1_c_p2 - p1; // should be -> p2
		const CPose3DRotVec  p2_c_p1_i_p2 = p2 + p1_i_p2; // Should be -> p1

        CMatrixDouble44 M1,M2;
        p1.getHomogeneousMatrix(M1);
        p2.getHomogeneousMatrix(M2);

        CPose3DQuat q1(M1), q2(M2), q1_c_q2(UNINITIALIZED_POSE);
        q1_c_q2 = q1 + q2;
        const CPose3DRotVec p3(q1_c_q2);

		EXPECT_NEAR(0, (p1_c_p2_i_p2.getAsVectorVal()-p2.getAsVectorVal()).array().abs().sum(), 1e-5)
			<< "p1          : " << p1 << endl
			<< "p2          : " << p2 << endl
			<< "p1_c_p2_i_p2: " << p1_c_p2_i_p2 << endl;

		EXPECT_NEAR(0, (p2_c_p1_i_p2.getAsVectorVal()-p1.getAsVectorVal()).array().abs().sum(), 1e-5)
			<< "p1          : " << p1 << endl
			<< "p2          : " << p2 << endl
			<< "p2 matrix   : " << endl << p2.getHomogeneousMatrixVal() << endl
			<< "p1_i_p2     : " << p1_i_p2 << endl
			<< "p1_i_p2 matrix: " << endl << p1_i_p2.getHomogeneousMatrixVal() << endl
			<< "p2_c_p1_i_p2: " << p2_c_p1_i_p2 << endl;

        EXPECT_NEAR(0, (p3.getAsVectorVal()-p1_c_p2.getAsVectorVal()).array().abs().sum(), 1e-5)
			<< "p3          : " << p3 << endl
			<< "p1_c_p2     : " << p1_c_p2 << endl;
	}
};

// Elemental tests:
TEST_F(Pose3DRotVecTests,DefaultValues)
{
	{
		CPose3DRotVec   p;
		test_default_values(p, "Default");
	}
}

TEST_F(Pose3DRotVecTests,Conversions)
{
    test_conversions();
}

TEST_F(Pose3DRotVecTests,Initialization)
{
	CPose3DRotVec p(0.2,0.3,0.4,1,2,3);
	EXPECT_NEAR(p.m_coords[0],1,  1e-7);
	EXPECT_NEAR(p.m_coords[1],2,  1e-7);
	EXPECT_NEAR(p.m_coords[2],3,  1e-7);
	EXPECT_NEAR(p.m_rotvec[0],0.2,  1e-7);
	EXPECT_NEAR(p.m_rotvec[1],0.3,  1e-7);
	EXPECT_NEAR(p.m_rotvec[2],0.4,  1e-7);
}

TEST_F(Pose3DRotVecTests,Compose)
{
	test_compose( 25.0, 2.0,  3.0, DEG2RAD(-30),DEG2RAD(89),DEG2RAD(0),
	             -10.0, 4.0, -8.0, DEG2RAD( 20),DEG2RAD( 9),DEG2RAD(0));
}
