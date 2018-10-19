/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/vision/TSimpleFeature.h>
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using mrpt::img::TPixelCoord;
using mrpt::vision::TSimpleFeatureList;
using std::vector;

#if MRPT_HAS_OPENCV

// Prototypes of functions exported from "vision/src/faster/*" to
// "vision/src/*":
void fast_corner_detect_9(
	const IplImage* I, TSimpleFeatureList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row);
void fast_corner_detect_10(
	const IplImage* I, TSimpleFeatureList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row);
void fast_corner_detect_12(
	const IplImage* I, TSimpleFeatureList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row);

#endif
