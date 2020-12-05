/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#ifndef MRPTPERFAPP_COMMON_H
#define MRPTPERFAPP_COMMON_H

#include <mrpt/core/bits_math.h>
#include <mrpt/img/CImage.h>
#include <mrpt/obs/obs_frwds.h>  // CObservation2DRangeScan
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/CTimeLogger.h>

#include <functional>
#include <list>

using namespace mrpt;
using namespace mrpt::img;
using namespace mrpt::io;
using namespace mrpt::serialization;
using mrpt::system::CTicTac;
using mrpt::system::CTimeLogger;

// All the register functions: --------------------
void register_tests_icpslam();
void register_tests_poses();
void register_tests_pose_interp();
void register_tests_matrices();
void register_tests_grids();
void register_tests_grid3D();
void register_tests_pointmaps();
void register_tests_random();
void register_tests_math();
void register_tests_image();
void register_tests_scan_matching();
void register_tests_feature_extraction();
void register_tests_feature_matching();
void register_tests_graph();
void register_tests_graphslam();
void register_tests_CObservation3DRangeScan();
void register_tests_atan2lut();
void register_tests_strings();
void register_tests_octomaps();
void register_tests_yaml();
// -------------------------------------------------

using TestFunctor =
	std::function<double(int, int)>;  // return run-time in secs.

struct TestData
{
	TestData(const char* nam, TestFunctor f, int a1 = 0, int a2 = 0)
		: name(nam), func(f), arg1(a1), arg2(a2)
	{
	}

	const char* name;
	TestFunctor func;
	int arg1, arg2;
};

// Common data & functions available to all performance modules:
extern std::list<TestData> lstTests;

void dummy_do_nothing_with_string(const std::string& s);
void getTestImage(unsigned int img_index, mrpt::img::CImage& out_img);

#endif
