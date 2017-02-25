/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef MRPTPERFAPP_COMMON_H
#define MRPTPERFAPP_COMMON_H

#include <mrpt/utils/CImage.h>
#include <mrpt/utils/CTicTac.h>
#include <list>

// All the register functions: --------------------
void register_tests_icpslam();
void register_tests_poses();
void register_tests_matrices();
void register_tests_grids();
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
// -------------------------------------------------

typedef double (*TestFunctor)(int a1, int a2);  // return run-time in secs.

struct TestData
{
	TestData(const char*nam,TestFunctor f,int a1=0,int a2=0) : name(nam),func(f),arg1(a1),arg2(a2) { }

	const char *name;
	TestFunctor func;
	int arg1,arg2;
};

// Common data & functions available to all performance modules:
extern std::list<TestData> lstTests;

extern const float SCAN_RANGES_1[361];
extern const char  SCAN_VALID_1[361];


void dummy_do_nothing_with_string(const std::string &s);
void getTestImage(unsigned int img_index, mrpt::utils::CImage &out_img );


#endif
