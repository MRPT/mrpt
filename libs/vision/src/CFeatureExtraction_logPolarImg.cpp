/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                                 |
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

#include <mrpt/vision.h>  // Precompiled headers

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

