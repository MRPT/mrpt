/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/vision/TKeyPoint.h>
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using mrpt::img::TPixelCoord;
using mrpt::vision::TKeyPointList;
using std::vector;

#if MRPT_HAS_OPENCV

// Prototypes of functions exported from "vision/src/faster/*" to
// "vision/src/*":
void fast_corner_detect_9(
	const cv::Mat& I, TKeyPointList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row);
void fast_corner_detect_10(
	const cv::Mat& I, TKeyPointList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row);
void fast_corner_detect_12(
	const cv::Mat& I, TKeyPointList& corners, int barrier, uint8_t octave,
	std::vector<size_t>* out_feats_index_by_row);

#endif
