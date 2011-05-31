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

#ifndef MRPTPERFAPP_COMMON_H
#define MRPTPERFAPP_COMMON_H

#include <mrpt/utils/CImage.h>

// All the register functions: --------------------
void register_tests_icpslam();
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
