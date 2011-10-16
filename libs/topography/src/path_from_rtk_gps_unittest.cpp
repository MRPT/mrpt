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


#include <mrpt/topography.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/filesystem.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::topography;
using namespace std;

// Defined in run_unittests.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}


TEST(TopographyReconstructPathFrom3RTK, sampleDataset )
{
	mrpt::poses::CPose3DInterpolator robot_path;

	mrpt::slam::CRawlog  rawlog;

	const string dataset_fil = MRPT_GLOBAL_UNITTEST_SRC_DIR + string("/share/mrpt/datasets/test_rtk_path.rawlog");
	if (!mrpt::system::fileExists(dataset_fil))
	{
		cerr << "WARNING: Skipping test due to missing file: " << dataset_fil << "\n";
		return;
	}
	if (!rawlog.loadFromRawLogFile(dataset_fil))
	{
		cerr << "WARNING: Skipping test due to error loading file: " << dataset_fil << "\n";
	}
	else
	{
		mrpt::topography::TPathFromRTKInfo	rtk_path_info;

		// -------------------------------------------
		// Run path reconstruction:
		// -------------------------------------------
		mrpt::topography::path_from_rtk_gps(
			robot_path,
			rawlog,
			0, // first entry
			rawlog.size()-1, // last entry
			false, // Isn't a GUI
			false, // disableGPSInterp
			1, // path_smooth_filter_size
			&rtk_path_info );

		EXPECT_EQ(robot_path.size(),75u);

		// Expected values:
		//1226225355.000000 279.705647 216.651473 8.517821 0.194222 -0.083873 -0.045293
		//1226225380.000000 377.095830 233.343569 9.724171 0.177037 -0.073565 -0.019024
		const mrpt::system::TTimeStamp t1 = mrpt::system::time_tToTimestamp( 1226225355.000000 );
		const mrpt::system::TTimeStamp t2 = mrpt::system::time_tToTimestamp( 1226225380.000000 );
		const CPose3D pose_GT_1(279.705647,216.651473,8.517821,0.194222,-0.083873,-0.045293);
		const CPose3D pose_GT_2(377.095830,233.343569,9.724171,0.177037,-0.073565,-0.019024);

		CPose3D pose1,pose2;
		bool    valid;
		robot_path.interpolate(t1,pose1,valid);
		EXPECT_TRUE(valid);

		robot_path.interpolate(t2,pose2,valid);
		EXPECT_TRUE(valid);


		vector_double p1vec(12), p2vec(12);
		pose1.getAs12Vector(p1vec);
		pose2.getAs12Vector(p2vec);

		vector_double p1vec_gt(12), p2vec_gt(12);
		pose_GT_1.getAs12Vector(p1vec_gt);
		pose_GT_2.getAs12Vector(p2vec_gt);

		EXPECT_NEAR( (p1vec - p1vec_gt).array().abs().sum(), 0, 1e-3);
		EXPECT_NEAR( (p2vec - p2vec_gt).array().abs().sum(), 0, 1e-3);
	}
}
