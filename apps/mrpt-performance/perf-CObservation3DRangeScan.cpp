/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/random.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/filesystem.h>

#include "common.h"

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::io;
using namespace mrpt::system;
using namespace mrpt::serialization;
using namespace std;
using namespace std::string_literals;

const string rgbd_test_rawlog_file =
	mrpt::system::getShareMRPTDir() + "datasets/tests_rgbd.rawlog"s;

void generateRandomMaskImage(
	mrpt::math::CMatrixF& m, const unsigned int nrows, const unsigned int ncols)
{
	m.resize(nrows, ncols);
	for (unsigned r = 0; r < nrows; r++)
		for (unsigned c = 0; c < ncols; c++)
			m(r, c) = static_cast<float>(
				mrpt::random::getRandomGenerator().drawUniform(0.0, 3.0));
}

double obs3d_test_depth_to_3d(int a, int b)
{
	CObservation3DRangeScan obs1;
	{
		CFileGZInputStream f(rgbd_test_rawlog_file);
		archiveFrom(f) >> obs1;
	}

	CTimeLogger timlog;

	T3DPointsProjectionParams pp;
	pp.USE_SSE2 = (a & 0x01) != 0;

	TRangeImageFilterParams fp;
	mrpt::math::CMatrixF minF, maxF;
	if (b & 0x01)
	{
		generateRandomMaskImage(
			minF, obs1.rangeImage.rows(), obs1.rangeImage.cols());
		fp.rangeMask_min = &minF;
	}
	if (b & 0x02)
	{
		generateRandomMaskImage(
			maxF, obs1.rangeImage.rows(), obs1.rangeImage.cols());
		fp.rangeMask_max = &maxF;
	}

	for (int i = 0; i < 100; i++)
	{
		CObservation3DRangeScan obs = obs1;
		// to avoid counting the generation of the LUT
		if (i > 0) timlog.enter("run");

		obs.unprojectInto(obs, pp, fp);

		if (i > 0) timlog.leave("run");
	}
	const double t = timlog.getMeanTime("run");
	timlog.clear(true);
	return t;
}

double obs3d_test_depth_to_2d_scan(int useMinFilter, int useMaxFilter)
{
	CObservation3DRangeScan obs1;
	{
		CFileGZInputStream f(rgbd_test_rawlog_file);
		archiveFrom(f) >> obs1;
	}

	CTimeLogger timlog;

	T3DPointsTo2DScanParams sp;
	sp.sensorLabel = "mysensor";

	TRangeImageFilterParams fp;
	mrpt::math::CMatrixF minF, maxF;
	if (useMinFilter)
	{
		generateRandomMaskImage(
			minF, obs1.rangeImage.rows(), obs1.rangeImage.cols());
		fp.rangeMask_min = &minF;
	}
	if (useMaxFilter)
	{
		generateRandomMaskImage(
			maxF, obs1.rangeImage.rows(), obs1.rangeImage.cols());
		fp.rangeMask_max = &maxF;
	}

	for (int i = 0; i < 10; i++)
	{
		CObservation3DRangeScan obs = obs1;
		CObservation2DRangeScan fake2d;
		timlog.enter("run");
		obs.convertTo2DScan(fake2d, sp, fp);
		timlog.leave("run");
	}
	const double t = timlog.getMeanTime("run");
	timlog.clear(true);
	return t;
}

// ------------------------------------------------------
// register_tests_CObservation3DRangeScan
// ------------------------------------------------------
void register_tests_CObservation3DRangeScan()
{
	if (mrpt::system::fileExists(rgbd_test_rawlog_file))
	{
		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->3D (w/o SSE2)", obs3d_test_depth_to_3d,
			0x00, 0);
		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->3D (w/SSE2)", obs3d_test_depth_to_3d,
			0x01, 0);

		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->3D (w/o SSE2,minFilter)",
			obs3d_test_depth_to_3d, 0x00, 0x01);
		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->3D (w/SSE2,minFilter)",
			obs3d_test_depth_to_3d, 0x01, 0x01);

		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->3D (w/o SSE2,maxFilter)",
			obs3d_test_depth_to_3d, 0x00, 0x02);
		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->3D (w/SSE2,maxFilter)",
			obs3d_test_depth_to_3d, 0x01, 0x02);

		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->3D (w/o SSE2,min/maxFilter)",
			obs3d_test_depth_to_3d, 0x00, 0x03);
		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->3D (w/SSE2,min/maxFilter)",
			obs3d_test_depth_to_3d, 0x01, 0x03);

		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->2D scan", obs3d_test_depth_to_2d_scan);
		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->2D scan + min_filter",
			obs3d_test_depth_to_2d_scan, 1, 0);
		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->2D scan + max_filter",
			obs3d_test_depth_to_2d_scan, 0, 1);
		lstTests.emplace_back(
			"3DRangeScan: 320x240 Depth->2D scan + min/max_filters",
			obs3d_test_depth_to_2d_scan, 1, 1);
	}
}
