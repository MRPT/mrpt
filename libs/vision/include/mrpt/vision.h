/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef __mrpt_vision_H
#define __mrpt_vision_H

#ifndef MRPT_NO_WARN_BIG_HDR
#include <mrpt/utils/core_defs.h>
MRPT_WARNING("Including <mrpt/vision.h> makes compilation much slower, consider including only what you need (define MRPT_NO_WARN_BIG_HDR to disable this warning)")
#endif

#include <mrpt/vision/utils.h>
#include <mrpt/vision/TSimpleFeature.h>
#include <mrpt/vision/multiDesc_utils.h>
#include <mrpt/vision/chessboard_camera_calib.h>
#include <mrpt/vision/chessboard_stereo_camera_calib.h>
#include <mrpt/vision/chessboard_find_corners.h>
#include <mrpt/vision/pinhole.h>
#include <mrpt/vision/CCamModel.h>
#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/CVideoFileWriter.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/vision/descriptor_kdtrees.h>
#include <mrpt/vision/descriptor_pairing.h>
#include <mrpt/vision/bundle_adjustment.h>
#include <mrpt/vision/CUndistortMap.h>
#include <mrpt/vision/CStereoRectifyMap.h>
#include <mrpt/vision/CImagePyramid.h>
#include <mrpt/vision/CDifodo.h>

// Maps:
#include <mrpt/maps/CLandmark.h>
#include <mrpt/maps/CLandmarksMap.h>

// Obs:
#include <mrpt/obs/CObservationVisualLandmarks.h>

#endif
