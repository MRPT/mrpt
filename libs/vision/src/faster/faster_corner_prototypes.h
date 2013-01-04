/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
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

#ifndef MRPT_FASTER_CORNER_PROTO_H
#define MRPT_FASTER_CORNER_PROTO_H

#include <mrpt/utils/types.h>
#include <mrpt/vision/TSimpleFeature.h>
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

using mrpt::utils::TPixelCoord;
using mrpt::vision::TSimpleFeatureList;
using std::vector;

#if MRPT_HAS_OPENCV

	// Prototypes of functions exported from "vision/src/faster/*" to "vision/src/*":
	void fast_corner_detect_9 (const IplImage* I, TSimpleFeatureList & corners, int barrier, uint8_t octave,std::vector<size_t> * out_feats_index_by_row);
	void fast_corner_detect_10(const IplImage* I, TSimpleFeatureList & corners, int barrier, uint8_t octave,std::vector<size_t> * out_feats_index_by_row);
	void fast_corner_detect_12(const IplImage* I, TSimpleFeatureList & corners, int barrier, uint8_t octave,std::vector<size_t> * out_feats_index_by_row);


	// Internal prototypes:
	void fast_corner_detect_plain_9 (const IplImage* i, TSimpleFeatureList &corners, int b, uint8_t octave,std::vector<size_t> * out_feats_index_by_row);
	void fast_corner_detect_plain_10(const IplImage* i, TSimpleFeatureList &corners, int b, uint8_t octave,std::vector<size_t> * out_feats_index_by_row);
	void fast_corner_detect_plain_12(const IplImage* i, TSimpleFeatureList &corners, int b, uint8_t octave,std::vector<size_t> * out_feats_index_by_row);

#endif


#endif
