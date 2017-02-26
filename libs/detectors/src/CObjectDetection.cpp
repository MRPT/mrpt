/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "detectors-precomp.h"  // Precompiled headers

#include <mrpt/detectors/CObjectDetection.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

using namespace mrpt::detectors;
using namespace mrpt::utils;

void CObjectDetection::detectObjects(const CImage *img, vector_detectable_object &detected)
{
	mrpt::obs::CObservationImage o;
	o.timestamp = mrpt::system::now();
	o.image.setFromImageReadOnly(*img);
	this->detectObjects_Impl(&o,detected);
}
