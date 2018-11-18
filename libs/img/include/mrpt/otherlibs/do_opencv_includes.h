/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#pragma once

// By including this file you make sure of #including all the relevant OpenCV
// headers, from OpenCV 1.0 up to the latest version.

#include <mrpt/config.h>

#if MRPT_HAS_OPENCV
// OPENCV HEADERS
#define CV_NO_CVV_IMAGE  // Avoid CImage name crash

#if MRPT_OPENCV_VERSION_NUM < 0x240
#error "MRPT requires OpenCV 2.4.0 or newer"
#endif

#if MRPT_OPENCV_VERSION_NUM > 0x300
#include <opencv2/core/fast_math.hpp>
#endif
#include <opencv2/opencv_modules.hpp>
// Core:
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
// Highgui:
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
// imgproc:
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
// features2d:
#include <opencv2/features2d/features2d.hpp>
// tracking:
#include <opencv2/video/tracking.hpp>
/// start added by Raghavender Sahdev
#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif
#ifdef HAVE_OPENCV_LINE_DESCRIPTOR
#include <opencv2/line_descriptor.hpp>
#endif
#ifdef HAVE_OPENCV_PLOT
#include <opencv2/plot.hpp>
#endif
/// end added by Raghavender Sahdev
#ifdef HAVE_OPENCV_IMGCODECS
#include <opencv2/imgcodecs.hpp>
#endif

#if MRPT_OPENCV_VERSION_NUM >= 0x300
#include <opencv2/video/tracking_c.h>
#endif
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#endif  // MRPT_HAS_OPENCV
