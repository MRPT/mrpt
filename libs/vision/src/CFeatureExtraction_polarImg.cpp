/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers

#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace std;

// Linear-Polar Transform
// J.L. Blanco, Apr 2009
// Code contributed to OpenCV 1.1.1 long time ago. Removed from MRPT now.

void CFeatureExtraction::internal_computePolarImageDescriptors(
	const mrpt::img::CImage& in_img, CFeatureList& in_features)
{
	MRPT_START
#if MRPT_HAS_OPENCV

	mrpt::system::CTimeLoggerEntry tle(
		profiler, "internal_computePolarImageDescriptors");

	ASSERT_(options.PolarImagesOptions.radius > 1);
	ASSERT_(options.PolarImagesOptions.bins_angle > 1);
	ASSERT_(options.PolarImagesOptions.bins_distance > 1);

	const unsigned int radius = options.PolarImagesOptions.radius;
	const unsigned int patch_w = options.PolarImagesOptions.bins_distance;
	const unsigned int patch_h = options.PolarImagesOptions.bins_angle;

	CImage linpolar_frame(patch_w, patch_h, in_img.getChannelCount());

	// Compute intensity-domain spin images
	for (auto& f : in_features)
	{
		// Overwrite scale with the descriptor scale:
		f.keypoint.octave = radius;

		const auto pt = cv::Point2f(f.keypoint.pt.x, f.keypoint.pt.y);

#if MRPT_OPENCV_VERSION_NUM < 0x300
		const cv::Mat& in = in_img.asCvMatRef();
		cv::Mat& out = linpolar_frame.asCvMatRef();
		cvLinearPolar(
			&in, &out, pt, radius, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);
#else
		cv::warpPolar(
			in_img.asCvMatRef(), linpolar_frame.asCvMatRef(),
			cv::Size(patch_w, patch_h), pt, radius,
			cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS);
#endif

		// Get the image as a matrix and save as patch:
		f.descriptors.PolarImg.emplace();
		linpolar_frame.getAsMatrix(*f.descriptors.PolarImg);

	}  // end for it

#else
	THROW_EXCEPTION("This method needs MRPT compiled with OpenCV support");
#endif
	MRPT_END
}
