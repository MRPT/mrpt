/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2012  University of Malaga                           |
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

#include <mrpt/vision.h>
#include <gtest/gtest.h>

#include "chessboard_stereo_camera_calib_internal.h"

using namespace mrpt;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::vision;
using namespace std;


class StereoCalibTests : public ::testing::Test {
protected:
	virtual void SetUp()
	{
	}
	virtual void TearDown() {  }

	void testStereoCalibJacobians(
		double x,double y, double z, double yaw, double pitch, double roll )
	{
		const CPose3D camPose = CPose3D(x2,y2,z2,yaw2,pitch2,roll2);

		// Prepare a test state:
		// --------------------------------------
		TCalibrationStereoImageList images;
		vector<size_t>   valid_image_pair_indices;
		vector<TPoint3D> obj_points;

		obj_points.push_back(TPoint3D(-0.4,0.3,0));
		valid_image_pair_indices.push_back(0);

		lm_stat_t  lm_stat(images, valid_image_pair_indices, obj_points);

		lm_stat.left_cam_poses.push_back( CPose3D(0,0,1) );

		// [fx fy cx cy k1 k2 k3 t1 t2]
		const double cam_l_params[9] = {600, 400, 320,240, 0,0,0,0,0 };
		const double cam_r_params[9] = {500, 300, 250,200, 0,0,0,0,0 };
		lm_stat.left_cam_params  = CArrayDouble<9>(cam_l_params);
		lm_stat.right_cam_params = CArrayDouble<9>(cam_r_params);


		// Evaluate theoretical Jacobians:
		TResidualJacobianList jacobs; // Theoretical output Jacobians
		mrpt::vision::recompute_errors_and_Jacobians( lm_stat, jacobs );


		// Compare:
		//EXPECT_NEAR(
	}

};


TEST_F(StereoCalibTests,Jacobians)
{
	testStereoCalibJacobians(0,0,0,DEG2RAD(0),DEG2RAD(0),DEG2RAD(0));

}

