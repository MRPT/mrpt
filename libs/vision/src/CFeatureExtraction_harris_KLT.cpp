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
using namespace mrpt::img;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace std;

/************************************************************************************************
 *							extractFeaturesKLT
 ************************************************************************************************/
void CFeatureExtraction::extractFeaturesKLT(
	const mrpt::img::CImage& inImg, CFeatureList& feats, unsigned int init_ID,
	unsigned int nDesiredFeatures, const TImageROI& ROI) const
{
	//#define VERBOSE_TIMING

#ifdef VERBOSE_TIMING
	CTicTac tictac;
#endif
	MRPT_START

#if MRPT_HAS_OPENCV
	const unsigned int MAX_COUNT = 300;

	// -----------------------------------------------------------------
	// Create OpenCV Local Variables
	// -----------------------------------------------------------------
	int count = 0;
	int nPts;

#ifdef VERBOSE_TIMING
	tictac.Tic();
#endif
	const cv::Mat img(cv::cvarrToMat(inImg.getAs<IplImage>()));

#ifdef VERBOSE_TIMING
	cout << "[KLT] Attach: " << tictac.Tac() * 1000.0f << endl;
#endif
	const CImage inImg_gray(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const cv::Mat cGrey(cv::cvarrToMat(inImg_gray.getAs<IplImage>()));

	nDesiredFeatures <= 0 ? nPts = MAX_COUNT : nPts = nDesiredFeatures;

#ifdef VERBOSE_TIMING
	tictac.Tic();
#endif

#ifdef VERBOSE_TIMING
	cout << "[KLT] Create: " << tictac.Tac() * 1000.0f << endl;
#endif
	count = nPts;  // Number of points to find

	// -----------------------------------------------------------------
	// Select good features with subpixel accuracy (USING HARRIS OR KLT)
	// -----------------------------------------------------------------
	const bool use_harris = (options.featsType == featHarris);

#ifdef VERBOSE_TIMING
	tictac.Tic();
#endif
	std::vector<cv::Point2f> points;
	cv::goodFeaturesToTrack(
		cGrey, points, nPts,
		(double)options.harrisOptions.threshold,  // for rejecting weak local
		// maxima ( with min_eig <
		// threshold*max(eig_image) )
		(double)options.harrisOptions
			.min_distance,  // minimum distance between features
		cv::noArray(),  // mask
		3,  // blocksize
		use_harris, /* harris */
		options.harrisOptions.k);
#ifdef VERBOSE_TIMING
	cout << "[KLT] Find feats: " << tictac.Tac() * 1000.0f << endl;
#endif

	if (nDesiredFeatures > 0 && count < nPts)
		cout << "\n[WARNING][selectGoodFeaturesKLT]: Only " << count << " of "
			 << nDesiredFeatures << " points could be extracted in the image."
			 << endl;

	if (options.FIND_SUBPIXEL)
	{
#ifdef VERBOSE_TIMING
		tictac.Tic();
#endif
		// Subpixel interpolation
		cv::cornerSubPix(
			cGrey, points, cv::Size(3, 3), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.05));

#ifdef VERBOSE_TIMING
		cout << "[KLT] subpixel: " << tictac.Tac() * 1000.0f << endl;
#endif
	}

// -----------------------------------------------------------------
// Fill output structure
// -----------------------------------------------------------------
#ifdef VERBOSE_TIMING
	tictac.Tic();
#endif

	feats.clear();
	unsigned int borderFeats = 0;
	unsigned int nCFeats = init_ID;
	int i = 0;
	const int limit = min(nPts, count);
	int offset = (int)this->options.patchSize / 2 + 1;
	unsigned int imgH = inImg.getHeight();
	unsigned int imgW = inImg.getWidth();

	while (i < limit)
	{
		const int xBorderInf = (int)floor(points[i].x - options.patchSize / 2);
		const int xBorderSup = (int)floor(points[i].x + options.patchSize / 2);
		const int yBorderInf = (int)floor(points[i].y - options.patchSize / 2);
		const int yBorderSup = (int)floor(points[i].y + options.patchSize / 2);

		if (options.patchSize == 0 ||
			((xBorderSup < (int)imgW) && (xBorderInf > 0) &&
			 (yBorderSup < (int)imgH) && (yBorderInf > 0)))
		{
			CFeature::Ptr ft = mrpt::make_aligned_shared<CFeature>();

			ft->type = featKLT;
			ft->x = points[i].x;  // X position
			ft->y = points[i].y;  // Y position
			ft->track_status = status_TRACKED;  // Feature Status
			ft->response = 0.0;  // A value proportional to the quality of the
			// feature (unused yet)
			ft->ID = nCFeats++;  // Feature ID into extraction
			ft->patchSize = options.patchSize;  // The size of the feature patch

			if (options.patchSize > 0)
			{
				inImg.extract_patch(
					ft->patch, round(ft->x) - offset, round(ft->y) - offset,
					options.patchSize,
					options.patchSize);  // Image patch surronding the feature
			}

			feats.push_back(ft);

		}  // end if
		else
			borderFeats++;

		i++;
	}  // end while

#ifdef VERBOSE_TIMING
	cout << "[KLT] Create output: " << tictac.Tac() * 1000.0f << endl;
#endif

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END

}  // end of function
