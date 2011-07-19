/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#ifndef MRPT_VISION_INTERNAL_OPECV_INCL_H
#define MRPT_VISION_INTERNAL_OPECV_INCL_H

#include <mrpt/config.h>

#if MRPT_HAS_OPENCV
	// OPENCV HEADERS
#	define CV_NO_CVV_IMAGE // Avoid CImage name crash

#	if MRPT_OPENCV_VERSION_NUM>=0x211
#		include <opencv2/core/core.hpp>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#		include <opencv2/calib3d/calib3d.hpp>
#	else
		// For OpenCV <=2.1
#		include <cv.h>
#		include <highgui.h>
#		include <cvaux.h>
#	endif

	#ifdef CImage	// For old OpenCV versions (<=1.0.0)
	#undef CImage
	#endif

	using mrpt::utils::CImage;

#endif // MRPT_HAS_OPENCV

#endif
