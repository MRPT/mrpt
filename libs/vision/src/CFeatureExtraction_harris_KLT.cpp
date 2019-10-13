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
	unsigned int nDesiredFeatures, const TImageROI& ROI)
{
	MRPT_START
	CTimeLoggerEntry tle(profiler, "extractFeaturesKLT");

#if MRPT_HAS_OPENCV
	const unsigned int MAX_COUNT = 300;

	// -----------------------------------------------------------------
	// Create OpenCV Local Variables
	// -----------------------------------------------------------------
	profiler.enter("extractFeaturesKLT.img2gray");

	const CImage inImg_gray(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const cv::Mat& cGrey = inImg_gray.asCvMatRef();

	profiler.leave("extractFeaturesKLT.img2gray");

	const auto nPts = (nDesiredFeatures <= 0) ? MAX_COUNT : nDesiredFeatures;

	// -----------------------------------------------------------------
	// Select good features with subpixel accuracy (USING HARRIS OR KLT)
	// -----------------------------------------------------------------
	const bool use_harris = (options.featsType == featHarris);

	std::vector<cv::Point2f> points;
	profiler.enter("extractFeaturesKLT.goodFeaturesToTrack");

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

	profiler.leave("extractFeaturesKLT.goodFeaturesToTrack");

	const unsigned int count = points.size();

	if (nDesiredFeatures > 0 && count < nPts)
		cout << "\n[WARNING][selectGoodFeaturesKLT]: Only " << count << " of "
			 << nDesiredFeatures << " points could be extracted in the image."
			 << endl;

	if (options.FIND_SUBPIXEL && !points.empty())
	{
		profiler.enter("extractFeaturesKLT.cornerSubPix");
		// Subpixel interpolation
		cv::cornerSubPix(
			cGrey, points, cv::Size(3, 3), cv::Size(-1, -1),
			cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 10, 0.05));

		profiler.leave("extractFeaturesKLT.cornerSubPix");
	}

	CTimeLoggerEntry tle2(profiler, "extractFeaturesKLT.fillFeatsStruct");

	feats.clear();
	unsigned int borderFeats = 0;
	unsigned int nCFeats = init_ID;
	int i = 0;
	const int limit = min(nPts, count);
	int offset = (int)this->options.patchSize / 2 + 1;
	unsigned int imgH = inImg.getHeight();
	unsigned int imgW = inImg.getWidth();

	const float W = options.patchSize * 0.5f;

	for (; i < limit; i++)
	{
		const int xBorderInf = (int)floor(points[i].x - W);
		const int xBorderSup = (int)floor(points[i].x + W);
		const int yBorderInf = (int)floor(points[i].y - W);
		const int yBorderSup = (int)floor(points[i].y + W);

		if (options.patchSize != 0 &&
			((xBorderSup >= (int)imgW) || (xBorderInf < 0) ||
			 (yBorderSup >= (int)imgH) || (yBorderInf < 0)))
		{
			borderFeats++;
			continue;
		}

		CFeature ft;

		ft.type = featKLT;
		ft.keypoint.pt.x = points[i].x;  // X position
		ft.keypoint.pt.y = points[i].y;  // Y position
		ft.track_status = status_TRACKED;  // Feature Status
		ft.response = 0.0;  // A value proportional to the quality of the
		// feature (unused yet)
		ft.keypoint.ID = nCFeats++;  // Feature ID into extraction
		ft.patchSize = options.patchSize;  // The size of the feature patch

		if (options.patchSize > 0)
		{
			ft.patch.emplace();
			inImg.extract_patch(
				*ft.patch, round(ft.keypoint.pt.x) - offset,
				round(ft.keypoint.pt.y) - offset, options.patchSize,
				options.patchSize);  // Image patch surronding the feature
		}

		feats.emplace_back(std::move(ft));

	}  // end while

#else
	THROW_EXCEPTION("The MRPT has been compiled with MRPT_HAS_OPENCV=0 !");
#endif

	MRPT_END

}  // end of function
