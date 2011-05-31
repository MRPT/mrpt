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

#include <mrpt/vision.h>  // Precompiled headers

#include <mrpt/vision/CFeatureExtraction.h>

#include "do_opencv_includes.h"


using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::system;
using namespace std;


/************************************************************************************************
								computeLogPolarImageDescriptors
************************************************************************************************/
void  CFeatureExtraction::internal_computeLogPolarImageDescriptors(
	const CImage	&in_img,
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

	CImage	logpolar_frame( patch_w, patch_h, in_img.getChannelCount() );

	// Compute intensity-domain spin images
	for (CFeatureList::iterator it=in_features.begin();it!=in_features.end();++it)
	{
		// Overwrite scale with the descriptor scale:
		(*it)->scale = radius;

		// Use OpenCV to convert:
		cvLogPolar(
			in_img.getAsIplImage(),
			logpolar_frame.getAsIplImage(),
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

