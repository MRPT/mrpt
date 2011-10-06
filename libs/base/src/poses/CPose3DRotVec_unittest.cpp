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



class Pose3DRotVecTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}

	virtual void TearDown() {  }

	void test_default_values(const CPose3DRotVec &p, const std::string & label)
	{
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

};

// Elemental tests:
TEST_F(Pose3DRotVecTests,DefaultValues)
{
	{
		CPose3DRotVec   p;
		test_default_values(p, "Default");
	}
}

TEST_F(Pose3DRotVecTests,Initialization)
{
    MRPT_TODO("Continue with unit tests")
//	Pose3DRotVecTests   p(1,2,3,0.2,0.3,0.4);
//	EXPECT_NEAR(p.x(),1,  1e-7);
//	EXPECT_NEAR(p.y(),2,  1e-7);
//	EXPECT_NEAR(p.z(),3,  1e-7);
//	EXPECT_NEAR(p.yaw(),0.2,  1e-7);
//	EXPECT_NEAR(p.pitch(),0.3,  1e-7);
//	EXPECT_NEAR(p.roll(),0.4,  1e-7);
}
