/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */

#pragma once

#ifndef MRPT_NO_WARN_BIG_HDR
MRPT_WARNING(
	"Including <mrpt/vision.h> makes compilation much slower, consider "
	"including only what you need (define MRPT_NO_WARN_BIG_HDR to disable this "
	"warning)")
#endif

#include <mrpt/vision/CDifodo.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/CImagePyramid.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <mrpt/vision/CUndistortMap.h>
#include <mrpt/vision/CVideoFileWriter.h>
#include <mrpt/vision/TKeyPoint.h>
#include <mrpt/vision/chessboard_camera_calib.h>
#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/vision/descriptor_kdtrees.h>
#include <mrpt/vision/descriptor_pairing.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/vision/utils.h>

// Maps:
#include <mrpt/maps/CLandmark.h>
#include <mrpt/maps/CLandmarksMap.h>

// Obs:
#include <mrpt/obs/CObservationVisualLandmarks.h>
