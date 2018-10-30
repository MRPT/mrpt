/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
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
	const mrpt::img::CImage& in_img, CFeatureList& in_features) const
{
	MRPT_START
#if MRPT_HAS_OPENCV

	ASSERT_(options.PolarImagesOptions.radius > 1);
	ASSERT_(options.PolarImagesOptions.bins_angle > 1);
	ASSERT_(options.PolarImagesOptions.bins_distance > 1);

	const unsigned int radius = options.PolarImagesOptions.radius;
	const unsigned int patch_w = options.PolarImagesOptions.bins_distance;
	const unsigned int patch_h = options.PolarImagesOptions.bins_angle;

	CImage linpolar_frame(patch_w, patch_h, in_img.getChannelCount());

	// Compute intensity-domain spin images
	for (auto it = in_features.begin(); it != in_features.end(); ++it)
	{
		// Overwrite scale with the descriptor scale:
		(*it)->scale = radius;

		cvLinearPolar(  // Use version sent to OpenCV
			in_img.getAs<IplImage>(), linpolar_frame.getAs<IplImage>(),
			cvPoint2D32f((*it)->x, (*it)->y), radius,
			CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);

		// Get the image as a matrix and save as patch:
		linpolar_frame.getAsMatrix((*it)->descriptors.PolarImg);

	}  // end for it

#else
	THROW_EXCEPTION("This method needs MRPT compiled with OpenCV support");
#endif
	MRPT_END
}
