/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/obs/CObservation3DRangeScan.h>

#include <gtest/gtest.h>

using namespace mrpt;
using namespace std;

// Defined in tests/test_main.cpp
namespace mrpt { namespace utils {
	extern std::string MRPT_GLOBAL_UNITTEST_SRC_DIR;
  }
}

void fillSampleObs(mrpt::obs::CObservation3DRangeScan &obs)
{
	obs.hasRangeImage = true;
	obs.rangeImage_setSize(160,320);

	obs.rangeImage.setZero();

	for (int r=10;r<16;r++)
		for (int c=10;c<=r;c++)
			obs.rangeImage(r,c) = r;
}


TEST(CObservation3DRangeScan, Project3D_noFilter)
{
	mrpt::obs::CObservation3DRangeScan  o;
	fillSampleObs(o);

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	o.project3DPointsFromDepthImageInto(o,pp,fp);

	EXPECT_EQ(o.points3D_x.size(),27);
	//cout << "pts:" <<  << endl;
}

TEST(CObservation3DRangeScan, Project3D_filterMinMax)
{
	mrpt::obs::CObservation3DRangeScan  o;
	fillSampleObs(o);

	mrpt::math::CMatrix fMax,fMin;
	fMax.resizeLike(o.rangeImage);
	fMin.resizeLike(o.rangeImage);

	fMin(12,12) = 11.5f; fMax(12,12) = 12.5f; // pass
	fMin(14,14) = 15.5f; fMax(14,14) = 16.5f; // don't pass

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;

	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	o.project3DPointsFromDepthImageInto(o,pp,fp);

	cout << "pts:" <<  o.points3D_x.size() << endl;
	EXPECT_EQ(o.points3D_x.size(),27);
}

