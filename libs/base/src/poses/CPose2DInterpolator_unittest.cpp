/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/system/datetime.h>
#include <gtest/gtest.h>


TEST(CPose2DInterpolator,interp)
{
	using namespace mrpt::poses;
	using mrpt::math::TPose2D;
	using mrpt::utils::DEG2RAD;

	const mrpt::system::TTimeStamp t0 = mrpt::system::now();
	const mrpt::system::TTimeStamp dt = mrpt::system::secondsToTimestamp(0.10);

	CPose2DInterpolator pose_path;

	pose_path.insert( t0, TPose2D(1.,2.,DEG2RAD(30.0)) );
	pose_path.insert( t0+2*dt, TPose2D(1.+3.,2.+4., DEG2RAD(30.0+20.0) ));

	TPose2D interp;
	bool valid;
	pose_path.interpolate(t0+dt,interp,valid);

	EXPECT_TRUE(valid);

	const TPose2D interp_good(1.+1.5,2.+2.0,DEG2RAD(30.0+10.0) );
	for (unsigned int i=0;i<interp_good.size();i++) {
		EXPECT_NEAR(interp_good[i], interp[i], 1e-4);
	}
}

