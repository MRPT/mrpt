/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace std;


/************************************************************************************************
								computeLogPolarImageDescriptors
************************************************************************************************/
void  CFeatureExtraction::internal_computeLogPolarImageDescriptors(
	const mrpt::utils::CImage	&in_img,
	CFeatureList		&in_features) const
{
	MRPT_START
#if MRPT_HAS_OPENCV

	ASSERT_(options.LogPolarImagesOptions.radius>1)
	ASSERT_(options.LogPolarImagesOptions.num_angles>1)
	ASSERT_(options.LogPolarImagesOptions.rho_scale>0)

	const unsigned int radius = options.LogPolarImagesOptions.radius;
	const unsigned int patch_h = options.LogPolarImagesOptions.num_angles;
	const double rho_scale = options.LogPolarImagesOptions.rho_scale;
	const unsigned int patch_w = rho_scale * std::log(static_cast<double>(radius));

	mrpt::utils::CImage	logpolar_frame( patch_w, patch_h, in_img.getChannelCount() );

	// Compute intensity-domain spin images
	for (CFeatureList::iterator it=in_features.begin();it!=in_features.end();++it)
	{
		// Overwrite scale with the descriptor scale:
		(*it)->scale = radius;

		// Use OpenCV to convert:
		cvLogPolar(
			in_img.getAs<IplImage>(),
			logpolar_frame.getAs<IplImage>(),
			cvPoint2D32f( (*it)->x,(*it)->y ),
			rho_scale,
			CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS );

		// Get the image as a matrix and save as patch:
		logpolar_frame.getAsMatrix( (*it)->descriptors.LogPolarImg );

	} // end for it


#else
		THROW_EXCEPTION("This method needs MRPT compiled with OpenCV support")
#endif
	MRPT_END
}

