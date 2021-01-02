/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "detectors-precomp.h"  // Precompiled headers

#include <mrpt/detectors/CObjectDetection.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>

// Universal include for all versions of OpenCV
#include <mrpt/3rdparty/do_opencv_includes.h>

using namespace mrpt::detectors;

void CObjectDetection::detectObjects(
	const mrpt::img::CImage* img, vector_detectable_object& detected)
{
	mrpt::obs::CObservationImage o;
	o.timestamp = mrpt::system::now();
	o.image = img->makeShallowCopy();
	this->detectObjects_Impl(o, detected);
}
