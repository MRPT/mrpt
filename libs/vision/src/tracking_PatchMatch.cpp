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

#include <mrpt/vision/tracking.h>
#include <mrpt/vision/CFeatureExtraction.h>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

/** Ctor  */
CFeatureTracker_PatchMatch::CFeatureTracker_PatchMatch(const mrpt::utils::TParametersDouble &extraParams) :
	CGenericFeatureTracker			( extraParams )
{
}


/** Track a set of features from old_img -> new_img by patch matching over a fixed window centered at each feature's previous location.
*  Optional parameters that can be passed in "extra_params":
*		- "window_width"  (Default=15)
*		- "window_height" (Default=15)
*		- "match_method" (Default=0)
*
*  Possible values for "match_method":
*		- 0 : Normalized cross correlation
*/
void CFeatureTracker_PatchMatch::trackFeatures_impl(
	const CImage &old_img,
	const CImage &new_img,
	vision::CFeatureList &featureList )
{
	MRPT_START
#if MRPT_HAS_OPENCV
#if MRPT_OPENCV_VERSION_NUM >= 0x211

	const unsigned int window_width = extra_params.getWithDefaultVal("window_width",15);
	const unsigned int window_height = extra_params.getWithDefaultVal("window_height",15);

	const double min_valid_matching = extra_params.getWithDefaultVal("min_valid_matching",0.97);
	const double min_matching_step = extra_params.getWithDefaultVal("min_matching_step",0.4);

	// Create a temporary gray image, if needed:
	const CImage new_img_gray(new_img, FAST_REF_OR_CONVERT_TO_GRAY);

	const size_t  new_img_width = new_img_gray.getWidth();
	const size_t  new_img_height = new_img_gray.getHeight();

	const size_t  patch_size = (!featureList.empty() && featureList[0]) ? featureList[0]->patch.getWidth() : 0;

	using namespace cv;

	// =======================================================================
	//                   OVERVIEW OF THE TRACKING ALGORITHM
	//
	// 1) For each existing feature from the last image, do:
	//     1.a) Compute the patch matching in a window around it
	//     1.b) If the response is good enough at the maximum, update the
	//          feature position. Otherwise, mark as "feature lost".
	// =======================================================================

	// For each old feature:
	for (size_t i=0;i<featureList.size();i++)
	{
		ASSERTDEB_(featureList[i].present())
		CFeature* feat = featureList[i].pointer();

		// Get the size of the patch so we know when we are too close to a border.
		ASSERTDEB_(feat->patch.getWidth()>1)
		ASSERTDEB_(feat->patch.getWidth()==feat->patch.getHeight())

		int x_search_ini = feat->x - window_width - (patch_size>>1);
		int	y_search_ini = feat->y - window_height - (patch_size>>1);
		int x_search_size = window_width*2;
		int	y_search_size = window_height*2;

		// Compute patch match:
		size_t  best_x, best_y;
		double  best_match;

#if 0
		mrpt::vision::openCV_cross_correlation(
			new_img_gray,
			feat->patch,
			best_x, best_y,best_match,   // Output
			x_search_ini, y_search_ini,
			x_search_size,y_search_size);
#else
		if ((x_search_ini + x_search_size  + patch_size)>new_img_width)
			x_search_size -= (x_search_ini + x_search_size + patch_size) - new_img_width;

		if ((y_search_ini + y_search_size  + patch_size)>new_img_height)
			y_search_size -= (y_search_ini + y_search_size  + patch_size) - new_img_height;

		ASSERT_( (x_search_ini + x_search_size + patch_size)<=new_img_width )
		ASSERT_( (y_search_ini + y_search_size + patch_size)<=new_img_height )

		const int result_width  = x_search_size+1;
		const int result_height = y_search_size+1;
		IplImage *result = cvCreateImage(cvSize(result_width,result_height),IPL_DEPTH_32F, 1);

		CImage  img_region_to_search;
		new_img_gray.extract_patch(
			img_region_to_search,
			x_search_ini,   // start corner
			y_search_ini ,
			patch_size+x_search_size,  // sub-image size
			patch_size+y_search_size
			);

		// Compute cross correlation:
		cvMatchTemplate(
			img_region_to_search.getAs<IplImage>(),
			feat->patch.getAs<IplImage>(),
			result,
			CV_TM_CCORR_NORMED
			//CV_TM_CCOEFF_NORMED
			);

		// Find the max point:
		CvPoint		max_point;  // In coords. relative to "result"!
		{
			double		mini;
			CvPoint		min_point;
			cvMinMaxLoc(result,&mini,&best_match,&min_point,&max_point,NULL);
			best_x = max_point.x+x_search_ini + (patch_size>>1);
			best_y = max_point.y+y_search_ini + (patch_size>>1);
		}
#endif
		//cout << "match: " << best_match << endl;

		if (best_match>min_valid_matching)
		{
			// Aditional checks:
			// A good, isolated match in its neighbourhood?
			CvScalar n1 = cvGet2D(result, std::max(0,max_point.y-1), max_point.x);
			CvScalar n2 = cvGet2D(result, std::min(result_height-1,max_point.y+1), max_point.x);
			CvScalar n3 = cvGet2D(result, max_point.y, std::max(0,max_point.x-1));
			CvScalar n4 = cvGet2D(result, max_point.y, std::min(result_width-1, max_point.x+1));

			CvScalar n5 = cvGet2D(result, std::max(0,max_point.y-1), std::max(0,max_point.x-1));
			CvScalar n6 = cvGet2D(result, std::max(0,max_point.y-1), std::min(result_width-1, max_point.x+1));
			CvScalar n7 = cvGet2D(result, std::min(result_height-1,max_point.y+1), std::max(0,max_point.x-1));
			CvScalar n8 = cvGet2D(result, std::min(result_height-1,max_point.y+1), std::min(result_width-1, max_point.x+1));

			// A local maxima?
			if (best_match>(n1.val[0]-min_matching_step) &&
				best_match>(n2.val[0]-min_matching_step) &&
				best_match>(n3.val[0]-min_matching_step) &&
				best_match>(n4.val[0]-min_matching_step) &&
				best_match>(n5.val[0]-min_matching_step) &&
				best_match>(n6.val[0]-min_matching_step) &&
				best_match>(n7.val[0]-min_matching_step) &&
				best_match>(n8.val[0]-min_matching_step) )
			{
				// OK: Accept it:
				feat->track_status	= status_TRACKED;
				feat->x				= best_x;
				feat->y				= best_y;
			}
		}
		else
		{
			// No potential match!
			// Mark as "lost":
			feat->track_status = status_LOST;
		}

		// Free memory:
		cvReleaseImage( &result );

	} // end for each prev feat.

#else
	THROW_EXCEPTION("This function requires OpenCV >= 2.1.1")
#endif
#else
	THROW_EXCEPTION("MRPT has been compiled without OpenCV!")
#endif
	MRPT_END
}


