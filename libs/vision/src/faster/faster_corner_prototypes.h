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

#ifndef MRPT_FASTER_CORNER_PROTO_H
#define MRPT_FASTER_CORNER_PROTO_H

#include <mrpt/utils/types.h>
#include <mrpt/vision/TSimpleFeature.h>
#include "../do_opencv_includes.h"

using mrpt::utils::TPixelCoord;
using mrpt::vision::TSimpleFeatureList;
using std::vector;

#if MRPT_HAS_OPENCV

	// Prototypes of functions exported from "vision/src/faster/*" to "vision/src/*":
	void fast_corner_detect_9 (const IplImage* I, TSimpleFeatureList & corners, int barrier, uint8_t octave);
	void fast_corner_detect_10(const IplImage* I, TSimpleFeatureList & corners, int barrier, uint8_t octave);
	void fast_corner_detect_12(const IplImage* I, TSimpleFeatureList & corners, int barrier, uint8_t octave);


	// Internal prototypes:
	void fast_corner_detect_plain_9 (const IplImage* i, TSimpleFeatureList &corners, int b, uint8_t octave);
	void fast_corner_detect_plain_10(const IplImage* i, TSimpleFeatureList &corners, int b, uint8_t octave);
	void fast_corner_detect_plain_12(const IplImage* i, TSimpleFeatureList &corners, int b, uint8_t octave);

#endif


#endif
