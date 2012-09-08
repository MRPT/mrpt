/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

#ifndef MRPT_DO_OPENCV_INCL_H
#define MRPT_DO_OPENCV_INCL_H

// By including this file you make sure of #including all the relevant OpenCV
// headers, from OpenCV 1.0 up to the latest version.

#include <mrpt/config.h>

#if MRPT_HAS_OPENCV
	// OPENCV HEADERS
#	define CV_NO_CVV_IMAGE // Avoid CImage name crash

#	if MRPT_OPENCV_VERSION_NUM>=0x211
#	if !defined(__cplusplus)
#		include <opencv2/core/core_c.h>
#		include <opencv2/highgui/highgui_c.h>
#		include <opencv2/imgproc/imgproc_c.h>
#	else
#		include <opencv2/core/core.hpp>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#		include <opencv2/features2d/features2d.hpp>
#		include <opencv2/video/tracking.hpp>
#		include <opencv2/calib3d/calib3d.hpp>
#		include <opencv2/objdetect/objdetect.hpp>

#		include <opencv2/legacy/legacy.hpp>  // CvImage
#		include <opencv2/legacy/compat.hpp>
#		if MRPT_OPENCV_VERSION_NUM>=0x240
#			include <opencv2/nonfree/nonfree.hpp>
#		endif
#	endif
#	else
		// For OpenCV <=2.1
#		include <cv.h>
#		include <highgui.h>
#		include <cvaux.h>
#	endif

	#ifdef CImage	// For old OpenCV versions (<=1.0.0)
	#undef CImage
	#endif

#	if defined(__cplusplus)
		#include <mrpt/utils/CImage.h>
		using mrpt::utils::CImage;

		typedef std::vector<CvPoint2D32f> CvPoint2D32fVector;
#	endif
#endif // MRPT_HAS_OPENCV

#endif
