/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
