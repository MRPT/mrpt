/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef MRPT_DO_OPENCV_INCL_H
#define MRPT_DO_OPENCV_INCL_H

// By including this file you make sure of #including all the relevant OpenCV
// headers, from OpenCV 1.0 up to the latest version.

#include <mrpt/config.h>

#if MRPT_HAS_OPENCV
	// OPENCV HEADERS
#	define CV_NO_CVV_IMAGE // Avoid CImage name crash

#	if MRPT_OPENCV_VERSION_NUM>=0x211
#		if MRPT_OPENCV_VERSION_NUM>0x300
#			include <opencv2/core/fast_math.hpp>
#		endif
#	if !defined(__cplusplus)
#		include <opencv2/core/core_c.h>
#		include <opencv2/highgui/highgui_c.h>
#		include <opencv2/imgproc/imgproc_c.h>
#	else
#		include <opencv2/core/core.hpp>
#		include <opencv2/core/core_c.h>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/highgui/highgui_c.h>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#		include <opencv2/features2d/features2d.hpp>
#		include <opencv2/video/tracking.hpp>
#		if MRPT_OPENCV_VERSION_NUM>=0x300
#			include <opencv2/video/tracking_c.h>
#		endif
#		include <opencv2/calib3d/calib3d.hpp>
#		include <opencv2/objdetect/objdetect.hpp>
//#		include <opencv2/core/eigen.hpp>  // Removed: opencv 2.3 had this file broken in Ubuntu 14.04 Trusty (??)
#		// These ones dissapeared in 3.0.0:  <opencv2/legacy/{legacy,compat}.hpp>  // CvImage
#		if (MRPT_OPENCV_VERSION_NUM>=0x240)
#		include <opencv2/opencv_modules.hpp>
#		endif
#	endif
#	else
		// For OpenCV <=2.1
#		include <cv.h>
#		include <highgui.h>
#		include <cvaux.h>
#	endif

#	if defined(__cplusplus)
		#include <mrpt/utils/CImage.h>
		using mrpt::utils::CImage;

		typedef std::vector<CvPoint2D32f> CvPoint2D32fVector;
#	endif
#endif // MRPT_HAS_OPENCV

#endif
