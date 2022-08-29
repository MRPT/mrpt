/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <gtest/gtest.h>
#include <mrpt/config.h>
#include <mrpt/containers/copy_container_typecasting.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/CHistogram.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

using namespace mrpt;
using namespace std;

// We need OPENCV to read the image internal to CObservation3DRangeScan,
// and to build the unprojected points LUTs, so skip tests if we don't have
// opencv.
#if MRPT_HAS_OPENCV

constexpr unsigned int TEST_RANGEIMG_WIDTH = 32;
constexpr unsigned int TEST_RANGEIMG_HEIGHT = 24;

constexpr float SECOND_LAYER_CONSTANT_RANGE = 50.0f;

void fillSampleObs(
	mrpt::obs::CObservation3DRangeScan& obs,
	mrpt::obs::T3DPointsProjectionParams& pp, int test_case)
{
	obs.hasRangeImage = true;

	// Create a second depth layer:
	obs.rangeImageOtherLayers.clear();
	mrpt::math::CMatrix_u16& ri_2nd = obs.rangeImageOtherLayers["LATEST"];

	obs.rangeImage_setSize(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);

	obs.rangeImage.setZero();

	obs.rangeUnits = 1e-3f;

	for (unsigned int r = 10; r < 16; r++)
		for (unsigned int c = 10; c <= r; c++)
			obs.rangeImage(r, c) = static_cast<uint16_t>(r / obs.rangeUnits);

	ri_2nd.fill(
		static_cast<uint16_t>(SECOND_LAYER_CONSTANT_RANGE / obs.rangeUnits));

	obs.cameraParams.ncols = TEST_RANGEIMG_WIDTH;
	obs.cameraParams.nrows = TEST_RANGEIMG_HEIGHT;
	obs.cameraParams.cx(TEST_RANGEIMG_WIDTH / 2);
	obs.cameraParams.cy(TEST_RANGEIMG_HEIGHT / 2);
	obs.cameraParams.fx(TEST_RANGEIMG_WIDTH * 2);
	obs.cameraParams.fy(TEST_RANGEIMG_WIDTH * 2);

	// Test case:
	pp.USE_SSE2 = (test_case & 1) != 0;
	pp.takeIntoAccountSensorPoseOnRobot = (test_case & 2) != 0;
}

TEST(CObservation3DRangeScan, Project3D_noFilter)
{
	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;

	for (int i = 0; i < 4; i++)	 // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);

		o.unprojectInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), 21U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMinMax1)
{
	mrpt::math::CMatrixF fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
		fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	fMin(12, 12) = 11.5f;
	fMax(12, 12) = 12.5f;  // pass
	fMin(14, 14) = 15.5f;
	fMax(14, 14) = 16.5f;  // don't pass
	// Rest of points: filter=0.0f -> no filtering

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i = 0; i < 8; i++)	 // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);
		fp.rangeCheckBetween = (i & 4) != 0;

		o.unprojectInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), 20U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_additionalLayers)
{
	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;

	pp.layer = "LATEST";

	for (int i = 0; i < 4; i++)	 // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);

		o.unprojectInto(o, pp, fp);
		EXPECT_EQ(
			o.points3D_x.size(), TEST_RANGEIMG_HEIGHT * TEST_RANGEIMG_WIDTH)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMinMaxAllBetween)
{
	mrpt::math::CMatrixF fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
		fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r = 10; r < 16; r++)
		for (int c = 10; c < 16; c++)
		{
			fMin(r, c) = r - 0.1f;	// All points actually lie in between
			fMax(r, c) = r + 0.1f;
		}

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i = 0; i < 16; i++)  // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);
		fp.rangeCheckBetween = (i & 8) != 0;

		o.unprojectInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), fp.rangeCheckBetween ? 21U : 0U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMinMaxNoneBetween)
{
	mrpt::math::CMatrixF fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH),
		fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r = 10; r < 16; r++)
		for (int c = 10; c < 16; c++)
		{
			fMin(r, c) = r + 1.1f;	// No point lies in between
			fMax(r, c) = r + 1.2f;
		}

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;
	fp.rangeMask_max = &fMax;

	for (int i = 0; i < 16; i++)  // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);
		fp.rangeCheckBetween = (i & 8) != 0;

		o.unprojectInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), fp.rangeCheckBetween ? 0U : 21U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMin)
{
	mrpt::math::CMatrixF fMin(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r = 10; r < 16; r++)
		for (int c = 10; c < 16; c++)
			fMin(r, c) =
				14.5f;	// Only last row of points should pass this filter

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_min = &fMin;

	for (int i = 0; i < 8; i++)	 // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);

		o.unprojectInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), 6U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, Project3D_filterMax)
{
	mrpt::math::CMatrixF fMax(TEST_RANGEIMG_HEIGHT, TEST_RANGEIMG_WIDTH);
	// Default filter=0.0f -> no filtering
	for (int r = 10; r < 16; r++)
		for (int c = 10; c < 16; c++)
			fMax(r, c) =
				11.5f;	// Only first 2 rows of points should pass this filter

	mrpt::obs::T3DPointsProjectionParams pp;
	mrpt::obs::TRangeImageFilterParams fp;
	fp.rangeMask_max = &fMax;

	for (int i = 0; i < 8; i++)	 // test all combinations of flags
	{
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);

		o.unprojectInto(o, pp, fp);
		EXPECT_EQ(o.points3D_x.size(), 3U)
			<< " testcase flags: i=" << i << std::endl;
	}
}

TEST(CObservation3DRangeScan, LoadAndCheckFloorPoints)
{
	const string rawlog_fil =
		UNITTEST_BASEDIR() + string("/tests/test-3d-obs-ground.rawlog");
	if (!mrpt::system::fileExists(rawlog_fil))
	{
		GTEST_FAIL() << "ERROR: test due to missing file: " << rawlog_fil
					 << "\n";
		return;
	}

	// Load sample 3D scan from file:
	mrpt::obs::CSensoryFrame sf;
	mrpt::io::CFileGZInputStream f(rawlog_fil);
	mrpt::serialization::archiveFrom(f) >> sf;

	auto obs = sf.getObservationByClass<mrpt::obs::CObservation3DRangeScan>();

	// Depth -> 3D points:
	mrpt::obs::T3DPointsProjectionParams pp;
	pp.takeIntoAccountSensorPoseOnRobot = true;
	mrpt::math::CHistogram hist(-0.15, 0.15, 30);
	std::vector<double> ptsz;
	std::vector<double> bin_x, bin_count;

	// decimation=1
	{
		mrpt::maps::CSimplePointsMap pts;
		obs->unprojectInto(pts, pp);

		mrpt::containers::copy_container_typecasting(
			pts.getPointsBufferRef_z(), ptsz);
		hist.add(ptsz);
		hist.getHistogram(bin_x, bin_count);

		/*
		//for (unsigned int i=0;i<bin_x.size();i++)
		//std::cout << "Hist[" << i << "] x=" << bin_x[i] << " count= " <<
		bin_count[i] << std::endl;
		*/

		EXPECT_LE(bin_count[11], 100);
		EXPECT_LE(bin_count[12], 1000);
		EXPECT_GE(bin_count[14], 12000);
		EXPECT_LE(bin_count[18], 700);
		EXPECT_LE(bin_count[19], 20);
	}

	// decimation=8
	{
		mrpt::maps::CSimplePointsMap pts;
		pp.decimation = 8;
		obs->unprojectInto(pts, pp);

		mrpt::containers::copy_container_typecasting(
			pts.getPointsBufferRef_z(), ptsz);
		hist.clear();
		hist.add(ptsz);
		hist.getHistogram(bin_x, bin_count);

		// for (unsigned int i = 0; i < bin_x.size(); i++)
		//	std::cout << "Hist[" << i << "] x=" << bin_x[i]
		//   << " count= " << bin_count[i] << std::endl;
		/*
		Hist[11] x=-0.0362069 count= 0
		Hist[12] x=-0.0258621 count= 7
		Hist[13] x=-0.0155172 count= 24
		Hist[14] x=-0.00517241 count= 130
		Hist[15] x=0.00517241 count= 234
		Hist[16] x=0.0155172 count= 129
		Hist[17] x=0.0258621 count= 85
		Hist[18] x=0.0362069 count= 32
		Hist[19] x=0.0465517 count= 3
		 */
		EXPECT_LE(bin_count[11], 2);
		EXPECT_LE(bin_count[12], 10);
		EXPECT_GE(bin_count[14], 100);
		EXPECT_LE(bin_count[18], 40);
		EXPECT_LE(bin_count[19], 5);
	}
}

TEST(CObservation3DRangeScan, SyntheticRange)
{
	for (int i = 0; i < 4; i++)	 // test all combinations of flags
	{
		mrpt::obs::T3DPointsProjectionParams pp;
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);
		const float R = 15.0f;
		o.rangeImage.fill(static_cast<uint16_t>(R / o.rangeUnits));

		// Ranges:
		o.range_is_depth = false;

		// x y z yaw pitch roll
		o.sensorPose = mrpt::poses::CPose3D::FromString("[1 2 3 0 0 0]");

		mrpt::maps::CSimplePointsMap pts;
		o.unprojectInto(pts, pp);

		EXPECT_EQ(pts.size(), TEST_RANGEIMG_WIDTH * TEST_RANGEIMG_HEIGHT);

		for (size_t j = 0; j < pts.size(); j++)
		{
			float px, py, pz;
			pts.getPoint(j, px, py, pz);

			mrpt::math::TPoint3D l;
			if (pp.takeIntoAccountSensorPoseOnRobot)
				o.sensorPose.inverseComposePoint(px, py, pz, l.x, l.y, l.z);
			else
				l = mrpt::math::TPoint3D(px, py, pz);

			EXPECT_NEAR(l.norm(), R, 2e-3);
		}
	}
}

TEST(CObservation3DRangeScan, SyntheticDepth)
{
	for (int i = 0; i < 4; i++)	 // test all combinations of flags
	{
		mrpt::obs::T3DPointsProjectionParams pp;
		mrpt::obs::CObservation3DRangeScan o;
		fillSampleObs(o, pp, i);
		const float R = 15.0f;
		o.rangeImage.fill(static_cast<uint16_t>(R / o.rangeUnits));

		// depth:
		o.range_is_depth = true;

		// x y z yaw pitch roll
		o.sensorPose = mrpt::poses::CPose3D::FromString("[1 2 3 0 0 0]");

		mrpt::maps::CSimplePointsMap pts;
		o.unprojectInto(pts, pp);

		EXPECT_EQ(pts.size(), TEST_RANGEIMG_WIDTH * TEST_RANGEIMG_HEIGHT);

		for (size_t j = 0; j < pts.size(); j++)
		{
			float px, py, pz;
			pts.getPoint(j, px, py, pz);

			mrpt::math::TPoint3D l;
			if (pp.takeIntoAccountSensorPoseOnRobot)
				o.sensorPose.inverseComposePoint(px, py, pz, l.x, l.y, l.z);
			else
				l = mrpt::math::TPoint3D(px, py, pz);

			EXPECT_NEAR(l.x, R, 2e-3);
		}
	}
}
#endif
