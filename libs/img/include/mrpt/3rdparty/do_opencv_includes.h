/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#pragma once

// By including this file you make sure of #including all the relevant OpenCV
// headers for OpenCV 2.4, 3.x, 4.x

#include <mrpt/config.h>

#if MRPT_HAS_OPENCV
// OPENCV HEADERS
#define CV_NO_CVV_IMAGE  // Avoid CImage name crash

#if MRPT_OPENCV_VERSION_NUM < 0x240
#error "MRPT requires OpenCV 2.4.0 or newer"
#endif

#include <opencv2/opencv_modules.hpp>

// C++ API:
#if MRPT_OPENCV_VERSION_NUM >= 0x300
// C++ API - opencv >=3
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/fast_math.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#else
// C++ API - opencv 2.4
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/tracking.hpp>
#endif

// C API:
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#ifdef HAVE_OPENCV_VIDEOIO
#include <opencv2/videoio/videoio_c.h>
#endif

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

// Backwards compatible macro:
#if MRPT_OPENCV_VERSION_NUM < 0x420
#define cvIplImage(X) (X)
#endif

#endif  // MRPT_HAS_OPENCV
