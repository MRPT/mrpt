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
#include <mrpt/3rdparty/do_opencv_includes.h>

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::system;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace std;

void CFeatureExtraction::extractFeaturesFAST(
	const mrpt::img::CImage& inImg, CFeatureList& feats, unsigned int init_ID,
	unsigned int nDesiredFeatures)
{
	MRPT_START

	mrpt::system::CTimeLoggerEntry tle(profiler, "extractFeaturesFAST");

#if MRPT_HAS_OPENCV
	using namespace cv;

	vector<KeyPoint> cv_feats;  // The opencv keypoint output vector

	// Make sure we operate on a gray-scale version of the image:
	const CImage inImg_gray(inImg, FAST_REF_OR_CONVERT_TO_GRAY);
	const Mat theImg = inImg_gray.asCvMat<cv::Mat>(SHALLOW_COPY);

#if MRPT_OPENCV_VERSION_NUM < 0x300
	FastFeatureDetector fastDetector(
		options.FASTOptions.threshold, options.FASTOptions.nonmax_suppression);
	fastDetector.detect(theImg, cv_feats);
#else
	Ptr<cv::FastFeatureDetector> fastDetector = cv::FastFeatureDetector::create(
		options.FASTOptions.threshold, options.FASTOptions.nonmax_suppression);
	fastDetector->detect(theImg, cv_feats);
#endif

	// *All* the features have been extracted.
	const size_t N = cv_feats.size();

	// Use KLT response instead of the OpenCV's original "response" field:
	if (options.FASTOptions.use_KLT_response)
	{
		const unsigned int KLT_half_win = 4;
		const unsigned int max_x = inImg_gray.getWidth() - 1 - KLT_half_win;
		const unsigned int max_y = inImg_gray.getHeight() - 1 - KLT_half_win;
		for (size_t i = 0; i < N; i++)
		{
			const unsigned int x = mrpt::round(cv_feats[i].pt.x);
			const unsigned int y = mrpt::round(cv_feats[i].pt.y);
			if (x > KLT_half_win && y > KLT_half_win && x <= max_x &&
				y <= max_y)
				cv_feats[i].response =
					inImg_gray.KLT_response(x, y, KLT_half_win);
			else
				cv_feats[i].response = -100;
		}
	}

	// Now:
	//  1) Sort them by "response": It's ~100 times faster to sort a list of
	//      indices "sorted_indices" than sorting directly the actual list of
	//      features "cv_feats"
	std::vector<size_t> sorted_indices(N);
	for (size_t i = 0; i < N; i++) sorted_indices[i] = i;
	std::sort(
		sorted_indices.begin(), sorted_indices.end(),
		KeypointResponseSorter<vector<KeyPoint>>(cv_feats));

	//  2) Filter by "min-distance" (in options.FASTOptions.min_distance)
	//  3) Convert to MRPT CFeatureList format.
	// Steps 2 & 3 are done together in the while() below.
	// The "min-distance" filter is done by means of a 2D binary matrix where
	// each cell is marked when one
	// feature falls within it. This is not exactly the same than a pure
	// "min-distance" but is pretty close
	// and for large numbers of features is much faster than brute force search
	// of kd-trees.
	// (An intermediate approach would be the creation of a mask image updated
	// for each accepted feature, etc.)

	const bool do_filter_min_dist = options.FASTOptions.min_distance > 1;

	// Used half the min-distance since we'll later mark as occupied the ranges
	// [i-1,i+1] for a feature at "i"
	const float occupied_grid_cell_size = options.FASTOptions.min_distance / 2;
	const float occupied_grid_cell_size_inv = 1.0f / occupied_grid_cell_size;

	unsigned int grid_lx =
		!do_filter_min_dist
			? 1
			: (unsigned int)(1 + inImg.getWidth() * occupied_grid_cell_size_inv);
	unsigned int grid_ly =
		!do_filter_min_dist
			? 1
			: (unsigned int)(1 + inImg.getHeight() * occupied_grid_cell_size_inv);

	mrpt::math::CMatrixBool occupied_sections(grid_lx, grid_ly);
	occupied_sections.fill(false);

	unsigned int nMax =
		(nDesiredFeatures != 0 && N > nDesiredFeatures) ? nDesiredFeatures : N;
	const int offset = (int)this->options.patchSize / 2 + 1;
	const size_t size_2 = options.patchSize / 2;
	const size_t imgH = inImg.getHeight();
	const size_t imgW = inImg.getWidth();
	unsigned int i = 0;
	unsigned int cont = 0;
	TFeatureID nextID = init_ID;

	if (!options.addNewFeatures) feats.clear();

	while (cont != nMax && i != N)
	{
		// Take the next feature fromt the ordered list of good features:
		const KeyPoint& kp = cv_feats[sorted_indices[i]];
		i++;

		// Patch out of the image??
		const int xBorderInf = (int)floor(kp.pt.x - size_2);
		const int xBorderSup = (int)floor(kp.pt.x + size_2);
		const int yBorderInf = (int)floor(kp.pt.y - size_2);
		const int yBorderSup = (int)floor(kp.pt.y + size_2);

		if (!(xBorderSup < (int)imgW && xBorderInf > 0 &&
			  yBorderSup < (int)imgH && yBorderInf > 0))
			continue;  // nope, skip.

		if (do_filter_min_dist)
		{
			// Check the min-distance:
			const auto sect_ix = size_t(kp.pt.x * occupied_grid_cell_size_inv);
			const auto sect_iy = size_t(kp.pt.y * occupied_grid_cell_size_inv);

			if (occupied_sections(sect_ix, sect_iy))
				continue;  // Already occupied! skip.

			// Mark section as occupied
			occupied_sections(sect_ix, sect_iy) = true;
			if (sect_ix > 0) occupied_sections(sect_ix - 1, sect_iy) = true;
			if (sect_iy > 0) occupied_sections(sect_ix, sect_iy - 1) = true;
			if (sect_ix < grid_lx - 1)
				occupied_sections(sect_ix + 1, sect_iy) = true;
			if (sect_iy < grid_ly - 1)
				occupied_sections(sect_ix, sect_iy + 1) = true;
		}

		// All tests passed: add new feature:
		CFeature ft;
		ft.type = featFAST;
		ft.keypoint.ID = nextID++;
		ft.keypoint.pt.x = kp.pt.x;
		ft.keypoint.pt.y = kp.pt.y;
		ft.response = kp.response;
		ft.orientation = kp.angle;
		ft.keypoint.octave = kp.octave;
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
		++cont;
	}

#endif
	MRPT_END
}
