/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "vision-precomp.h"  // Precompiled headers
//
#include <stack>  // Precompiled headers

// Note for MRPT: what follows below is a modified part of the "OCamCalib
// Toolbox":
//  See:
//  http://robotics.ethz.ch/~scaramuzza/Davide_Scaramuzza_files/Research/OcamCalib_Tutorial.htm
// Modifications include:
//  - Clean up of code and update to use STL containers, and smart pointers.
//  - Addition of a new method to detect a number of checkerboards.
//  - Modification of the dilation algorithm - see do_special_dilation().
//
// Original copyright note:
/************************************************************************************\
	This is improved variant of chessboard corner detection algorithm that
	uses a graph of connected quads. It is based on the code contributed
	by Vladimir Vezhnevets and Philip Gruebele.
	Here is the copyright notice from the original Vladimir's code:
	===============================================================

	The algorithms developed and implemented by Vezhnevets Vldimir
	aka Dead Moroz (vvp@graphics.cs.msu.ru)
	See http://graphics.cs.msu.su/en/research/calibration/opencv.html
	for detailed information.

	Reliability additions and modifications made by Philip Gruebele.
	<a href="mailto:pgruebele@cox.net">pgruebele@cox.net</a>

	His code was adapted for use with low resolution and omnidirectional cameras
	by Martin Rufli during his Master Thesis under supervision of Davide
Scaramuzza, at the ETH Zurich. Further enhancements include:
		- Increased chance of correct corner matching.
		- Corner matching over all dilation runs.

If you use this code, please cite the following articles:

1. Scaramuzza, D., Martinelli, A. and Siegwart, R. (2006), A Toolbox for Easily
Calibrating Omnidirectional Cameras, Proceedings of the IEEE/RSJ International
Conference on Intelligent Robots and Systems  (IROS 2006), Beijing, China,
October 2006.
2. Scaramuzza, D., Martinelli, A. and Siegwart, R., (2006). "A Flexible
Technique for Accurate Omnidirectional Camera Calibration and Structure from
Motion", Proceedings of IEEE International Conference of Vision Systems
(ICVS'06), New York, January 5-7, 2006.
3. Rufli, M., Scaramuzza, D., and Siegwart, R. (2008), Automatic Detection of
Checkerboards on Blurred and Distorted Images, Proceedings of the IEEE/RSJ
International Conference on Intelligent Robots and Systems (IROS 2008), Nice,
France, September 2008.

\************************************************************************************/

#include <array>
#include <map>

#include "checkerboard_ocamcalib_detector.h"

#if MRPT_HAS_OPENCV

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace std;

//===========================================================================
// CODE STARTS HERE
//===========================================================================
// Defines
#define MAX_CONTOUR_APPROX 7

// JL: Refactored code from within cvFindChessboardCorners3() and alternative
// algorithm:
bool do_special_dilation(
	CImage& thresh_img, const int dilations, IplConvKernel* kernel_cross,
	IplConvKernel* kernel_rect, IplConvKernel* kernel_diag1,
	IplConvKernel* kernel_diag2, IplConvKernel* kernel_horz,
	IplConvKernel* kernel_vert)
{
	cv::Mat m = thresh_img.asCvMat<cv::Mat>(SHALLOW_COPY);
	IplImage i = cvIplImage(m);
	IplImage* ipl = &i;

	bool isLast = false;

	switch (dilations)
	{
		case 37:
			cvDilate(ipl, ipl, kernel_cross, 1);
			isLast = true;
			[[fallthrough]];
		case 36:
			cvErode(ipl, ipl, kernel_rect, 1);
			[[fallthrough]];
		case 35:
			cvDilate(ipl, ipl, kernel_vert, 1);
			[[fallthrough]];
		case 34:
			cvDilate(ipl, ipl, kernel_vert, 1);
			[[fallthrough]];
		case 33:
			cvDilate(ipl, ipl, kernel_vert, 1);
			[[fallthrough]];
		case 32:
			cvDilate(ipl, ipl, kernel_vert, 1);
			[[fallthrough]];
		case 31:
			cvDilate(ipl, ipl, kernel_vert, 1);
			break;

		case 30:
			cvDilate(ipl, ipl, kernel_cross, 1);
			[[fallthrough]];
		case 29:
			cvErode(ipl, ipl, kernel_rect, 1);
			[[fallthrough]];
		case 28:
			cvDilate(ipl, ipl, kernel_horz, 1);
			[[fallthrough]];
		case 27:
			cvDilate(ipl, ipl, kernel_horz, 1);
			[[fallthrough]];
		case 26:
			cvDilate(ipl, ipl, kernel_horz, 1);
			[[fallthrough]];
		case 25:
			cvDilate(ipl, ipl, kernel_horz, 1);
			[[fallthrough]];
		case 24:
			cvDilate(ipl, ipl, kernel_horz, 1);
			break;

		case 23:
			cvDilate(ipl, ipl, kernel_diag2, 1);
			[[fallthrough]];
		case 22:
			cvDilate(ipl, ipl, kernel_diag1, 1);
			[[fallthrough]];
		case 21:
			cvDilate(ipl, ipl, kernel_diag2, 1);
			[[fallthrough]];
		case 20:
			cvDilate(ipl, ipl, kernel_diag1, 1);
			[[fallthrough]];
		case 19:
			cvDilate(ipl, ipl, kernel_diag2, 1);
			[[fallthrough]];
		case 18:
			cvDilate(ipl, ipl, kernel_diag1, 1);
			break;

		case 17:
			cvDilate(ipl, ipl, kernel_diag2, 1);
			[[fallthrough]];
		case 16:
			cvDilate(ipl, ipl, kernel_diag2, 1);
			[[fallthrough]];
		case 15:
			cvDilate(ipl, ipl, kernel_diag2, 1);
			[[fallthrough]];
		case 14:
			cvDilate(ipl, ipl, kernel_diag2, 1);
			break;

		case 13:
			cvDilate(ipl, ipl, kernel_diag1, 1);
			[[fallthrough]];
		case 12:
			cvDilate(ipl, ipl, kernel_diag1, 1);
			[[fallthrough]];
		case 11:
			cvDilate(ipl, ipl, kernel_diag1, 1);
			[[fallthrough]];
		case 10:
			cvDilate(ipl, ipl, kernel_diag1, 1);
			break;

		case 9:
			cvDilate(ipl, ipl, kernel_cross, 1);
			[[fallthrough]];
		case 8:
			cvErode(ipl, ipl, kernel_rect, 1);
			[[fallthrough]];
		case 7:
			cvDilate(ipl, ipl, kernel_cross, 1);
			[[fallthrough]];
		case 6:
			cvDilate(ipl, ipl, kernel_diag2, 1);
			isLast = true;
			[[fallthrough]];
		case 5:
			cvDilate(ipl, ipl, kernel_diag1, 1);
			[[fallthrough]];
		case 4:
			cvDilate(ipl, ipl, kernel_rect, 1);
			[[fallthrough]];
		case 3:
			cvErode(ipl, ipl, kernel_cross, 1);
			[[fallthrough]];
		case 2:
			cvDilate(ipl, ipl, kernel_rect, 1);
			[[fallthrough]];
		case 1:
			cvDilate(ipl, ipl, kernel_cross, 1);
			[[fallthrough]];
		case 0: /* first try: do nothing to the image */
			break;
	};

	return isLast;
}

//===========================================================================
// MAIN FUNCTION
//===========================================================================
// Return: -1: errors, 0: not found, 1: found OK
int cvFindChessboardCorners3(
	const CImage& img_, CvSize pattern_size,
	std::vector<CvPoint2D32f>& out_corners)
{
	// PART 0: INITIALIZATION
	//-----------------------------------------------------------------------
	// Initialize variables
	int flags = 1;  // not part of the function call anymore!
	size_t max_count = 0;
	int max_dilation_run_ID = -1;
	// const int min_dilations		=  0; // JL: was: 1
	// const int max_dilations		=  23; // JL: see do_special_dilation()
	int found = 0;

	vector<CvCBQuad::Ptr> quads;  // CvCBQuad **quads = 0;
	vector<CvCBQuad::Ptr> quad_group;  // CvCBQuad **quad_group		=  0;
	vector<CvCBCorner::Ptr> corners;  // CvCBCorner *corners			=  0;
	vector<CvCBQuad::Ptr>
		output_quad_group;  //	CvCBQuad **output_quad_group = 0;

	// debug trial. Martin Rufli, 28. Ocober, 2008
	int block_size = 0;

	// Further initializations
	int quad_count, group_idx;

	if (pattern_size.width < 2 || pattern_size.height < 2)
	{
		std::cerr << "Pattern should have at least 2x2 size" << endl;
		return -1;
	}
	if (pattern_size.width > 127 || pattern_size.height > 127)
	{
		std::cerr << "Pattern should not have a size larger than 127 x 127"
				  << endl;
		return -1;
	}

	// Assure it's a grayscale image:
	const CImage img(img_, FAST_REF_OR_CONVERT_TO_GRAY);

	CImage thresh_img(
		img.getWidth(), img.getHeight(),
		CH_GRAY);  // = cvCreateMat( img->rows, img->cols, CV_8UC1 );
	CImage thresh_img_save(
		img.getWidth(), img.getHeight(),
		CH_GRAY);  //  = cvCreateMat( img->rows, img->cols, CV_8UC1 );

	// JL: Move these constructors out of the loops:
	IplConvKernel* kernel_cross =
		cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_CROSS, nullptr);
	IplConvKernel* kernel_rect =
		cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT, nullptr);

	static int kernel_diag1_vals[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	IplConvKernel* kernel_diag1 = cvCreateStructuringElementEx(
		3, 3, 1, 1, CV_SHAPE_CUSTOM, kernel_diag1_vals);
	static int kernel_diag2_vals[9] = {0, 0, 1, 0, 1, 0, 1, 0, 0};
	IplConvKernel* kernel_diag2 = cvCreateStructuringElementEx(
		3, 3, 1, 1, CV_SHAPE_CUSTOM, kernel_diag2_vals);
	static int kernel_horz_vals[9] = {0, 0, 0, 1, 1, 1, 0, 0, 0};
	IplConvKernel* kernel_horz = cvCreateStructuringElementEx(
		3, 3, 1, 1, CV_SHAPE_CUSTOM, kernel_horz_vals);
	static int kernel_vert_vals[9] = {0, 1, 0, 0, 1, 0, 0, 1, 0};
	IplConvKernel* kernel_vert = cvCreateStructuringElementEx(
		3, 3, 1, 1, CV_SHAPE_CUSTOM, kernel_vert_vals);

	// For image binarization (thresholding)
	// we use an adaptive threshold with a gaussian mask
	// ATTENTION: Gaussian thresholding takes MUCH more time than Mean
	// thresholding!
	block_size = cvRound(MIN(img.getWidth(), img.getHeight()) * 0.2) | 1;

	cv::adaptiveThreshold(
		img.asCvMat<cv::Mat>(SHALLOW_COPY),
		thresh_img.asCvMat<cv::Mat>(SHALLOW_COPY), 255,
		CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, block_size, 0);

	thresh_img_save = thresh_img.makeDeepCopy();

	// PART 1: FIND LARGEST PATTERN
	//-----------------------------------------------------------------------
	// Checker patterns are tried to be found by dilating the background and
	// then applying a canny edge finder on the closed contours (checkers).
	// Try one dilation run, but if the pattern is not found, repeat until
	// max_dilations is reached.
	// for( int dilations = min_dilations; dilations <= max_dilations;
	// dilations++ )

	bool last_dilation = false;

	for (int dilations = 0; !last_dilation; dilations++)
	{
		// Calling "cvCopy" again is much faster than rerunning
		// "cvAdaptiveThreshold"
		thresh_img = thresh_img_save.makeDeepCopy();

		// Dilate squares:
		last_dilation = do_special_dilation(
			thresh_img, dilations, kernel_cross, kernel_rect, kernel_diag1,
			kernel_diag2, kernel_horz, kernel_vert);

		// In order to find rectangles that go to the edge, we draw a white
		// line around the image edge. Otherwise FindContours will miss those
		// clipped rectangle contours. The border color will be the image mean,
		// because otherwise we risk screwing up filters like cvSmooth()
		cv::rectangle(
			thresh_img.asCvMatRef(), cv::Point(0, 0),
			cv::Point(thresh_img.getWidth() - 1, thresh_img.getHeight() - 1),
			CV_RGB(255, 255, 255), 3, 8);

		// Generate quadrangles in the following function
		// "quad_count" is the number of cound quadrangles
		quad_count = icvGenerateQuads(
			quads, corners, thresh_img, flags, dilations, true);
		if (quad_count <= 0) continue;

		// The following function finds and assigns neighbor quads to every
		// quadrangle in the immediate vicinity fulfilling certain
		// prerequisites
		mrFindQuadNeighbors2(quads, dilations);

		// The connected quads will be organized in groups. The following loop
		// increases a "group_idx" identifier.
		// The function "icvFindConnectedQuads assigns all connected quads
		// a unique group ID.
		// If more quadrangles were assigned to a given group (i.e. connected)
		// than are expected by the input variable "pattern_size", the
		// function "icvCleanFoundConnectedQuads" erases the surplus
		// quadrangles by minimizing the convex hull of the remaining pattern.
		for (group_idx = 0;; group_idx++)
		{
			icvFindConnectedQuads(quads, quad_group, group_idx, dilations);

			if (quad_group.empty()) break;

			icvCleanFoundConnectedQuads(quad_group, pattern_size);
			size_t count = quad_group.size();

			// MARTIN's Code
			// To save computational time, only proceed, if the number of
			// found quads during this dilation run is larger than the
			// largest previous found number
			if (count /*>=*/ > max_count)
			{
				// cout << "CHECKERBOARD: Best found at dilation=" << dilations
				// << endl;

				// set max_count to its new value
				max_count = count;
				max_dilation_run_ID = dilations;

				// The following function labels all corners of every quad
				// with a row and column entry.
				// "count" specifies the number of found quads in "quad_group"
				// with group identifier "group_idx"
				// The last parameter is set to "true", because this is the
				// first function call and some initializations need to be
				// made.
				mrLabelQuadGroup(quad_group, pattern_size, true);

				// END------------------------------------------------------------------------

				// Allocate memory
				// output_quad_group.resize( (pattern_size.height+2) *
				// (pattern_size.width+2) ); // = (CvCBQuad**)cvAlloc(
				// sizeof(output_quad_group[0]) * ((pattern_size.height+2) *
				// (pattern_size.width+2)) );
				// The following function copies every member of "quad_group"
				// to "output_quad_group", because "quad_group" will be
				// overwritten during the next loop pass.
				// "output_quad_group" is a true copy of "quad_group" and
				// later used for output
				output_quad_group = quad_group;  // mrCopyQuadGroup( quad_group,
				// output_quad_group, max_count
				// );
			}
		}
	}

	// If enough corners have been found already, then there is no need for PART
	// 2 ->EXIT
	// JLBC for MRPT: Don't save to Matlab files (mrWriteCorners), but to
	// "CvPoint2D32f *out_corners":
	// Return true on success in finding all the quads.
	found = myQuads2Points(output_quad_group, pattern_size, out_corners);

	if (found != -1 && found != 1)
	{
		// PART 2: AUGMENT LARGEST PATTERN
		//-----------------------------------------------------------------------
		// Instead of saving all found quads of all dilation runs from PART 1,
		// we
		// just recompute them again, but skipping the dilation run which
		// produced the maximum number of found quadrangles.
		// In essence the first section of PART 2 is identical to the first
		// section of PART 1.
		// for( int dilations = max_dilations; dilations >= min_dilations;
		// dilations-- )
		last_dilation = false;
		for (int dilations = 0; !last_dilation; dilations++)
		{
			// if(max_dilation_run_ID == dilations)
			//	continue;

			// Calling "cvCopy" again is much faster than rerunning
			// "cvAdaptiveThreshold"
			thresh_img = thresh_img_save.makeDeepCopy();

			// Dilate squares:
			last_dilation = do_special_dilation(
				thresh_img, dilations, kernel_cross, kernel_rect, kernel_diag1,
				kernel_diag2, kernel_horz, kernel_vert);

			cv::rectangle(
				thresh_img.asCvMatRef(), cv::Point(0, 0),
				cv::Point(
					thresh_img.getWidth() - 1, thresh_img.getHeight() - 1),
				CV_RGB(255, 255, 255), 3, 8);

			quad_count = icvGenerateQuads(
				quads, corners, thresh_img, flags, dilations, false);
			if (quad_count <= 0) continue;

			// END------------------------------------------------------------------------

			// MARTIN's Code
			// The following loop is executed until no more newly found quads
			// can be matched to one of the border corners of the largest found
			// pattern from PART 1.
			// The function "mrAugmentBestRun" tests whether a quad can be
			// linked
			// to the existng pattern.
			// The function "mrLabelQuadGroup" then labels the newly added
			// corners
			// with the respective row and column entries.
			int feedBack = -1;
			while (feedBack == -1)
			{
				feedBack = mrAugmentBestRun(
					quads, dilations, output_quad_group, max_dilation_run_ID);

				// if we have found a new matching quad
				if (feedBack == -1)
				{
					// increase max_count by one
					max_count = max_count + 1;
					mrLabelQuadGroup(output_quad_group, pattern_size, false);

					// write the found corners to output array
					// Go to //__END__, if enough corners have been found
					found = myQuads2Points(
						output_quad_group, pattern_size, out_corners);
					// found = mrWriteCorners( output_quad_group, max_count,
					// pattern_size, min_number_of_corners);
					if (found == -1 || found == 1)
					{
						// JL: was a "goto exit;", but, have you seen
						// http://xkcd.com/292/ ??? ;-)
						last_dilation =
							true;  // This will break the outer for loop
						break;
					}
				}
			}

		}  // end for "dilations"

	}  // JL: Was label "exit:", but again, http://xkcd.com/292/ ;-)

	// Free mem:
	cvReleaseStructuringElement(&kernel_cross);
	cvReleaseStructuringElement(&kernel_rect);
	cvReleaseStructuringElement(&kernel_diag1);
	cvReleaseStructuringElement(&kernel_diag2);
	cvReleaseStructuringElement(&kernel_horz);
	cvReleaseStructuringElement(&kernel_vert);

	/*
	// MARTIN:
	found = mrWriteCorners( output_quad_group, max_count, pattern_size,
	min_number_of_corners);
	*/

	// If a linking problem was encountered, throw an error message
	if (found == -1)
	{
		std::cerr << "While linking the corners a problem was encountered. No "
					 "corner sequence is returned. "
				  << endl;
		return -1;
	}

	// Return found
	// Found can have the values
	// -1  ->	Error or corner linking problem, see std::cerr
	//  0  ->	Not enough corners were found
	//  1  ->	Enough corners were found
	return found;
}

double triangleArea(
	double x0, double y0, double x1, double y1, double x2, double y2)
{
	return std::abs(0.5 * (x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1)));
}

double median(const std::vector<double>& vec)
{
	std::vector<double> v = vec;  // Copy for sorting
	const size_t n = v.size() / 2;
	nth_element(v.begin(), v.begin() + n, v.end());
	return v[n];
}

//===========================================================================
// ERASE OVERHEAD
//===========================================================================
// If we found too many connected quads, remove those which probably do not
// belong.
void icvCleanFoundConnectedQuads(
	std::vector<CvCBQuad::Ptr>& quad_group, const CvSize& pattern_size)
{
	cv::MemStorage temp_storage;  // JL: "Modernized" to use C++ STL stuff.

	CvPoint2D32f center = cvPoint2D32f(0, 0);

	// Number of quads this pattern should contain
	const size_t expected_quads_count =
		((pattern_size.width + 1) * (pattern_size.height + 1) + 1) / 2;

	// If we have more quadrangles than we should, try to eliminate duplicates
	// or ones which don't belong to the pattern rectangle. Else go to the end
	// of the function
	const size_t nQuads = quad_group.size();
	if (nQuads <= expected_quads_count) return;  // Nothing to be done.

	// Create an array of quadrangle centers
	vector<CvPoint2D32f> centers(nQuads);
	temp_storage = cv::MemStorage(cvCreateMemStorage(0));

	// make also the list of squares areas, so we can discriminate by
	// too-large/small quads:
	// (Added by JLBC, JUN-2014)
	std::vector<double> quad_areas(nQuads);
	double min_area = DBL_MAX, max_area = -DBL_MAX, mean_area = 0.0;

	for (size_t i = 0; i < nQuads; i++)
	{
		CvPoint2D32f ci = cvPoint2D32f(0, 0);
		CvCBQuad::Ptr& q = quad_group[i];

		for (size_t j = 0; j < 4; j++)
		{
			CvPoint2D32f pt = q->corners[j]->pt;
			ci.x += pt.x;
			ci.y += pt.y;
		}

		ci.x *= 0.25f;
		ci.y *= 0.25f;

		// Quad area:
		const double a =
			triangleArea(
				q->corners[0]->pt.x, q->corners[0]->pt.y, q->corners[1]->pt.x,
				q->corners[1]->pt.y, q->corners[2]->pt.x, q->corners[2]->pt.y) +
			triangleArea(
				q->corners[0]->pt.x, q->corners[0]->pt.y, q->corners[2]->pt.x,
				q->corners[2]->pt.y, q->corners[3]->pt.x, q->corners[3]->pt.y);

		q->area = a;
		quad_areas[i] = a;
		mean_area += a;
		if (a < min_area) min_area = a;
		if (a > max_area) max_area = a;

		// Centers(i), is the geometric center of quad(i)
		// Center, is the center of all found quads
		centers[i] = ci;
		center.x += ci.x;
		center.y += ci.y;
	}
	center.x /= nQuads;
	center.y /= nQuads;
	mean_area /= nQuads;
	const double median_area = median(quad_areas);

	// ration: area/median:
	for (size_t i = 0; i < nQuads; i++)
	{
		quad_group[i]->area_ratio = quad_group[i]->area / median_area;
	}

	// If we have more quadrangles than we should, we try to eliminate bad
	// ones based on minimizing the bounding box. We iteratively remove the
	// point which reduces the size of the bounding box of the blobs the most
	// (since we want the rectangle to be as small as possible) remove the
	// quadrange that causes the biggest reduction in pattern size until we
	// have the correct number

	// JLBC (2014): Additional preliminary filter: remove quads that are too
	// small or too large

	// In the following, use "quad_group.size()" since the size will change with
	// iterations
	while (quad_group.size() > expected_quads_count)
	{
		double min_box_area = DBL_MAX;
		int min_box_area_index = -1;

		// For each point, check area:
		int most_outlier_idx = -1;
		double most_outlier_ratio = 1.0;
		for (size_t skip = 0; skip < quad_group.size(); skip++)
		{
			double ar = quad_group[skip]->area_ratio;
			if (ar > 1.0) ar = 1.0 / ar;

			if (ar < most_outlier_ratio)
			{
				most_outlier_ratio = ar;
				most_outlier_idx = skip;
			}
		}

		if (most_outlier_idx >= 0)
		{
			min_box_area_index = most_outlier_idx;
		}

		if (min_box_area_index == -1)  // if the previous filter didn't work:
		{
			// For each point, calculate box area without that point
			for (size_t skip = 0; skip < quad_group.size(); skip++)
			{
				// get bounding rectangle
				CvPoint2D32f temp = centers[skip];
				centers[skip] = center;
				CvMat pointMat =
					cvMat(1, quad_group.size(), CV_32FC2, &centers[0]);
				CvSeq* hull =
					cvConvexHull2(&pointMat, temp_storage, CV_CLOCKWISE, 1);
				centers[skip] = temp;
				double hull_area = fabs(cvContourArea(hull, CV_WHOLE_SEQ));

				// remember smallest box area
				if (hull_area < min_box_area)
				{
					min_box_area = hull_area;
					min_box_area_index = skip;
				}
				cvClearMemStorage(temp_storage);
			}
		}

		CvCBQuad::Ptr& q0 = quad_group[min_box_area_index];

		// remove any references to this quad as a neighbor
		for (size_t i = 0; i < quad_group.size(); i++)
		{
			CvCBQuad::Ptr& q = quad_group[i];

			for (size_t j = 0; j < 4; j++)
			{
				if (q->neighbors[j] == q0)
				{
					q->neighbors[j].reset();  // = 0;
					q->count--;
					for (size_t k = 0; k < 4; k++)
						if (q0->neighbors[k] == q)
						{
							q0->neighbors[k].reset();  // = 0;
							q0->count--;
							break;
						}
					break;
				}
			}
		}

		// remove the quad by copying th last quad in the list into its place
		quad_group.erase(quad_group.begin() + min_box_area_index);
		centers.erase(centers.begin() + min_box_area_index);
	}
}

//===========================================================================
// FIND COONECTED QUADS
//===========================================================================
void icvFindConnectedQuads(
	std::vector<CvCBQuad::Ptr>& quad, std::vector<CvCBQuad::Ptr>& out_group,
	const int group_idx, [[maybe_unused]] const int dilation)
{
	// initializations
	out_group.clear();

	const size_t quad_count = quad.size();

	// Scan the array for a first unlabeled quad
	for (size_t i = 0; i < quad_count; i++)
	{
		if (quad[i]->count < 0 || quad[i]->group_idx >= 0) continue;

		// Recursively find a group of connected quads starting from the seed
		// quad[i]
		CvCBQuad::Ptr& q = quad[i];

		std::stack<CvCBQuad::Ptr> seqStack;

		seqStack.push(q);  // cvSeqPush( stack, &q );

		q->group_idx = group_idx;
		out_group.push_back(q);  // out_group[count++] = q;

		while (!seqStack.empty())
		{
			q = seqStack.top();
			seqStack.pop();  // cvSeqPop( stack, &q );

			for (size_t k = 0; k < 4; k++)
			{
				CvCBQuad::Ptr& neighbor = q->neighbors[k];

				// If he neighbor exists and the neighbor has more than 0
				// neighbors and the neighbor has not been classified yet.
				if (neighbor && neighbor->count > 0 && neighbor->group_idx < 0)
				{
					neighbor->group_idx = group_idx;
					seqStack.push(neighbor);  // cvSeqPush( stack, &neighbor );
					out_group.push_back(
						neighbor);  // out_group[count++] = neighbor;
				}
			}
		}

		break;
	}
}

//===========================================================================
// LABEL CORNER WITH ROW AND COLUMN //DONE
//===========================================================================
void mrLabelQuadGroup(
	std::vector<CvCBQuad::Ptr>& quad_group, const CvSize& pattern_size,
	bool firstRun)
{
	const size_t count = quad_group.size();

	// If this is the first function call, a seed quad needs to be selected
	if (firstRun == true)
	{
		// Search for the (first) quad with the maximum number of neighbors
		// (usually 4). This will be our starting point.
		int max_id = -1;
		int max_number = -1;
		for (size_t i = 0; i < count; i++)
		{
			CvCBQuad* q = quad_group[i].get();
			if (q->count > max_number)
			{
				max_number = q->count;
				max_id = i;

				if (max_number == 4) break;
			}
		}

		// Mark the starting quad's (per definition) upper left corner with
		//(0,0) and then proceed clockwise
		// The following labeling sequence enshures a "right coordinate system"
		CvCBQuad* q = quad_group[max_id].get();

		q->labeled = true;
		q->corners[0]->row = 0;
		q->corners[0]->column = 0;
		q->corners[1]->row = 0;
		q->corners[1]->column = 1;
		q->corners[2]->row = 1;
		q->corners[2]->column = 1;
		q->corners[3]->row = 1;
		q->corners[3]->column = 0;
	}

	// Mark all other corners with their respective row and column
	bool flag_changed = true;
	while (flag_changed == true)
	{
		// First reset the flag to "false"
		flag_changed = false;

		// Go through all quads top down is faster, since unlabeled quads will
		// be inserted at the end of the list
		for (int i = int(count - 1); i >= 0; i--)
		{
			// Check whether quad "i" has been labeled already
			if ((quad_group[i])->labeled == false)
			{
				// Check its neighbors, whether some of them have been labeled
				// already
				for (size_t j = 0; j < 4; j++)
				{
					// Check whether the neighbor exists (i.e. is not the NULL
					// pointer)
					if ((quad_group[i])->neighbors[j])
					{
						CvCBQuad::Ptr& quadNeighborJ =
							quad_group[i]->neighbors[j];

						// Only proceed, if neighbor "j" was labeled
						if (quadNeighborJ->labeled == true)
						{
							// For every quad it could happen to pass here
							// multiple times. We therefore "break" later.
							// Check whitch of the neighbors corners is
							// connected to the current quad
							int connectedNeighborCornerId = -1;
							for (int k = 0; k < 4; k++)
							{
								if (quadNeighborJ->neighbors[k] ==
									quad_group[i])
								{
									connectedNeighborCornerId = k;

									// there is only one, therefore
									break;
								}
							}

							// For the following calculations we need the row
							// and column of the connected neighbor corner and
							// all other corners of the connected quad "j",
							// clockwise (CW)
							CvCBCorner::Ptr& conCorner =
								quadNeighborJ
									->corners[connectedNeighborCornerId];
							CvCBCorner::Ptr& conCornerCW1 =
								quadNeighborJ->corners
									[(connectedNeighborCornerId + 1) % 4];
							CvCBCorner::Ptr& conCornerCW2 =
								quadNeighborJ->corners
									[(connectedNeighborCornerId + 2) % 4];
							CvCBCorner::Ptr& conCornerCW3 =
								quadNeighborJ->corners
									[(connectedNeighborCornerId + 3) % 4];

							(quad_group[i])->corners[j]->row = conCorner->row;
							(quad_group[i])->corners[j]->column =
								conCorner->column;
							(quad_group[i])->corners[(j + 1) % 4]->row =
								conCorner->row - conCornerCW2->row +
								conCornerCW3->row;
							(quad_group[i])->corners[(j + 1) % 4]->column =
								conCorner->column - conCornerCW2->column +
								conCornerCW3->column;
							(quad_group[i])->corners[(j + 2) % 4]->row =
								conCorner->row + conCorner->row -
								conCornerCW2->row;
							(quad_group[i])->corners[(j + 2) % 4]->column =
								conCorner->column + conCorner->column -
								conCornerCW2->column;
							(quad_group[i])->corners[(j + 3) % 4]->row =
								conCorner->row - conCornerCW2->row +
								conCornerCW1->row;
							(quad_group[i])->corners[(j + 3) % 4]->column =
								conCorner->column - conCornerCW2->column +
								conCornerCW1->column;

							// Mark this quad as labeled
							(quad_group[i])->labeled = true;

							// Changes have taken place, set the flag
							flag_changed = true;

							// once is enough!
							break;
						}
					}
				}
			}
		}
	}

	// All corners are marked with row and column
	// Record the minimal and maximal row and column indices
	// It is unlikely that more than 8bit checkers are used per dimension, if
	// there are
	// an error would have been thrown at the beginning of
	// "cvFindChessboardCorners2"
	int min_row = 127;
	int max_row = -127;
	int min_column = 127;
	int max_column = -127;

	for (size_t i = 0; i < count; i++)
	{
		const CvCBQuad::Ptr& q = quad_group[i];

		for (size_t j = 0; j < 4; j++)
		{
			if ((q->corners[j])->row > max_row) max_row = (q->corners[j])->row;

			if ((q->corners[j])->row < min_row) min_row = (q->corners[j])->row;

			if ((q->corners[j])->column > max_column)
				max_column = (q->corners[j])->column;

			if ((q->corners[j])->column < min_column)
				min_column = (q->corners[j])->column;
		}
	}

	// Label all internal corners with "needsNeighbor" = false
	// Label all external corners with "needsNeighbor" = true,
	// except if in a given dimension the pattern size is reached
	for (int i = min_row; i <= max_row; i++)
	{
		for (int j = min_column; j <= max_column; j++)
		{
			// A flag that indicates, wheter a row/column combination is
			// executed multiple times
			bool flagg = false;

			// Remember corner and quad
			int cornerID = 0;
			int quadID = 0;

			for (size_t k = 0; k < count; k++)
			{
				for (size_t l = 0; l < 4; l++)
				{
					if (((quad_group[k])->corners[l]->row == i) &&
						((quad_group[k])->corners[l]->column == j))
					{
						if (flagg == true)
						{
							// Passed at least twice through here
							(quad_group[k])->corners[l]->needsNeighbor = false;
							(quad_group[quadID])
								->corners[cornerID]
								->needsNeighbor = false;
						}
						else
						{
							// Mark with needs a neighbor, but note the
							// address
							(quad_group[k])->corners[l]->needsNeighbor = true;
							cornerID = l;
							quadID = k;
						}

						// set the flag to true
						flagg = true;
					}
				}
			}
		}
	}

	// Complete Linking:
	// sometimes not all corners were properly linked in "mrFindQuadNeighbors2",
	// but after labeling each corner with its respective row and column, it is
	// possible to match them anyway.
	for (int i = min_row; i <= max_row; i++)
	{
		for (int j = min_column; j <= max_column; j++)
		{
			// the following "number" indicates the number of corners which
			// correspond to the given (i,j) value
			// 1	is a border corner or a conrer which still needs a neighbor
			// 2	is a fully connected internal corner
			// >2	something went wrong during labeling, report a warning
			int number = 1;

			// remember corner and quad
			int cornerID = 0;
			int quadID = 0;

			for (size_t k = 0; k < count; k++)
			{
				for (size_t l = 0; l < 4; l++)
				{
					if (((quad_group[k])->corners[l]->row == i) &&
						((quad_group[k])->corners[l]->column == j))
					{
						if (number == 1)
						{
							// First corner, note its ID
							cornerID = l;
							quadID = k;
						}

						else if (number == 2)
						{
							// Second corner, check wheter this and the
							// first one have equal coordinates, else
							// interpolate
							float delta_x =
								(quad_group[k])->corners[l]->pt.x -
								(quad_group[quadID])->corners[cornerID]->pt.x;
							float delta_y =
								(quad_group[k])->corners[l]->pt.y -
								(quad_group[quadID])->corners[cornerID]->pt.y;

							if (delta_x != 0 || delta_y != 0)
							{
								// Interpolate
								(quad_group[k])->corners[l]->pt.x =
									(quad_group[k])->corners[l]->pt.x -
									delta_x / 2;
								(quad_group[quadID])->corners[cornerID]->pt.x =
									(quad_group[quadID])
										->corners[cornerID]
										->pt.x +
									delta_x / 2;
								(quad_group[k])->corners[l]->pt.y =
									(quad_group[k])->corners[l]->pt.y -
									delta_y / 2;
								(quad_group[quadID])->corners[cornerID]->pt.y =
									(quad_group[quadID])
										->corners[cornerID]
										->pt.y +
									delta_y / 2;
							}
						}
						else if (number > 2)
						{
							// Something went wrong during row/column labeling
							// Report a Warning
							// ->Implemented in the function "mrWriteCorners"
						}

						// increase the number by one
						number = number + 1;
					}
				}
			}
		}
	}

	// Bordercorners don't need any neighbors, if the pattern size in the
	// respective direction is reached
	// The only time we can make shure that the target pattern size is reached
	// in a given
	// dimension, is when the larger side has reached the target size in the
	// maximal
	// direction, or if the larger side is larger than the smaller target size
	// and the
	// smaller side equals the smaller target size
	int largerDimPattern = max(pattern_size.height, pattern_size.width);
	int smallerDimPattern = min(pattern_size.height, pattern_size.width);
	bool flagSmallerDim1 = false;
	bool flagSmallerDim2 = false;

	if ((largerDimPattern + 1) == max_column - min_column)
	{
		flagSmallerDim1 = true;
		// We found out that in the column direction the target pattern size is
		// reached
		// Therefore border column corners do not need a neighbor anymore
		// Go through all corners
		for (size_t k = 0; k < count; k++)
		{
			for (size_t l = 0; l < 4; l++)
			{
				if ((quad_group[k])->corners[l]->column == min_column ||
					(quad_group[k])->corners[l]->column == max_column)
				{
					// Needs no neighbor anymore
					(quad_group[k])->corners[l]->needsNeighbor = false;
				}
			}
		}
	}

	if ((largerDimPattern + 1) == max_row - min_row)
	{
		flagSmallerDim2 = true;
		// We found out that in the column direction the target pattern size is
		// reached
		// Therefore border column corners do not need a neighbor anymore
		// Go through all corners
		for (size_t k = 0; k < count; k++)
		{
			for (size_t l = 0; l < 4; l++)
			{
				if ((quad_group[k])->corners[l]->row == min_row ||
					(quad_group[k])->corners[l]->row == max_row)
				{
					// Needs no neighbor anymore
					(quad_group[k])->corners[l]->needsNeighbor = false;
				}
			}
		}
	}

	// Check the two flags:
	//	-	If one is true and the other false, then the pattern target
	//		size was reached in in one direction -> We can check, whether the
	// target
	//		pattern size is also reached in the other direction
	//  -	If both are set to true, then we deal with a square board -> do
	//  nothing
	//  -	If both are set to false -> There is a possibility that the larger
	//  side is
	//		larger than the smaller target size -> Check and if true, then check
	// whether
	//		the other side has the same size as the smaller target size
	if ((flagSmallerDim1 == false && flagSmallerDim2 == true))
	{
		// Larger target pattern size is in row direction, check wheter smaller
		// target
		// pattern size is reached in column direction
		if ((smallerDimPattern + 1) == max_column - min_column)
		{
			for (size_t k = 0; k < count; k++)
			{
				for (int l = 0; l < 4; l++)
				{
					if ((quad_group[k])->corners[l]->column == min_column ||
						(quad_group[k])->corners[l]->column == max_column)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if ((flagSmallerDim1 == true && flagSmallerDim2 == false))
	{
		// Larger target pattern size is in column direction, check wheter
		// smaller target
		// pattern size is reached in row direction
		if ((smallerDimPattern + 1) == max_row - min_row)
		{
			for (size_t k = 0; k < count; k++)
			{
				for (size_t l = 0; l < 4; l++)
				{
					if ((quad_group[k])->corners[l]->row == min_row ||
						(quad_group[k])->corners[l]->row == max_row)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if ((flagSmallerDim1 == false && flagSmallerDim2 == false) &&
		smallerDimPattern + 1 < max_column - min_column)
	{
		// Larger target pattern size is in column direction, check wheter
		// smaller target
		// pattern size is reached in row direction
		if ((smallerDimPattern + 1) == max_row - min_row)
		{
			for (size_t k = 0; k < count; k++)
			{
				for (size_t l = 0; l < 4; l++)
				{
					if ((quad_group[k])->corners[l]->row == min_row ||
						(quad_group[k])->corners[l]->row == max_row)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}

	if ((flagSmallerDim1 == false && flagSmallerDim2 == false) &&
		smallerDimPattern + 1 < max_row - min_row)
	{
		// Larger target pattern size is in row direction, check wheter smaller
		// target
		// pattern size is reached in column direction
		if ((smallerDimPattern + 1) == max_column - min_column)
		{
			for (size_t k = 0; k < count; k++)
			{
				for (size_t l = 0; l < 4; l++)
				{
					if ((quad_group[k])->corners[l]->column == min_column ||
						(quad_group[k])->corners[l]->column == max_column)
					{
						// Needs no neighbor anymore
						(quad_group[k])->corners[l]->needsNeighbor = false;
					}
				}
			}
		}
	}
}

//===========================================================================
// GIVE A GROUP IDX
//===========================================================================
// This function replaces mrFindQuadNeighbors, which in turn replaced
// icvFindQuadNeighbors
void mrFindQuadNeighbors2(std::vector<CvCBQuad::Ptr>& quads, int dilation)
{
	// Thresh dilation is used to counter the effect of dilation on the
	// distance between 2 neighboring corners. Since the distance below is
	// computed as its square, we do here the same. Additionally, we take the
	// conservative assumption that dilation was performed using the 3x3 CROSS
	// kernel, which coresponds to the 4-neighborhood.
	const float thresh_dilation = (float)(2 * dilation + 3) *
								  (2 * dilation + 3) *
								  2;  // the "*2" is for the x and y component
	float dx, dy, dist;
	// int cur_quad_group = -1;

	const size_t quad_count = quads.size();

	// Find quad neighbors
	for (size_t idx = 0; idx < quad_count; idx++)
	{
		CvCBQuad::Ptr& cur_quad = quads[idx];

		// Go through all quadrangles and label them in groups
		// For each corner of this quadrangle
		for (size_t i = 0; i < 4; i++)
		{
			CvPoint2D32f pt;
			float min_dist = FLT_MAX;
			int closest_corner_idx = -1;
			CvCBQuad::Ptr closest_quad;

			if (cur_quad->neighbors[i]) continue;

			pt = cur_quad->corners[i]->pt;

			// Find the closest corner in all other quadrangles
			for (size_t k = 0; k < quad_count; k++)
			{
				if (k == idx) continue;

				for (size_t j = 0; j < 4; j++)
				{
					// If it already has a neighbor
					if (quads[k]->neighbors[j]) continue;

					dx = pt.x - quads[k]->corners[j]->pt.x;
					dy = pt.y - quads[k]->corners[j]->pt.y;
					dist = dx * dx + dy * dy;

					// The following "if" checks, whether "dist" is the
					// shortest so far and smaller than the smallest
					// edge length of the current and target quads
					if (dist < min_dist &&
						dist <= (cur_quad->edge_len + thresh_dilation) &&
						dist <= (quads[k]->edge_len + thresh_dilation))
					{
						// First Check everything from the viewpoint of the
						// current quad
						// compute midpoints of "parallel" quad sides 1
						float x1 = (cur_quad->corners[i]->pt.x +
									cur_quad->corners[(i + 1) % 4]->pt.x) /
								   2;
						float y1 = (cur_quad->corners[i]->pt.y +
									cur_quad->corners[(i + 1) % 4]->pt.y) /
								   2;
						float x2 = (cur_quad->corners[(i + 2) % 4]->pt.x +
									cur_quad->corners[(i + 3) % 4]->pt.x) /
								   2;
						float y2 = (cur_quad->corners[(i + 2) % 4]->pt.y +
									cur_quad->corners[(i + 3) % 4]->pt.y) /
								   2;
						// compute midpoints of "parallel" quad sides 2
						float x3 = (cur_quad->corners[i]->pt.x +
									cur_quad->corners[(i + 3) % 4]->pt.x) /
								   2;
						float y3 = (cur_quad->corners[i]->pt.y +
									cur_quad->corners[(i + 3) % 4]->pt.y) /
								   2;
						float x4 = (cur_quad->corners[(i + 1) % 4]->pt.x +
									cur_quad->corners[(i + 2) % 4]->pt.x) /
								   2;
						float y4 = (cur_quad->corners[(i + 1) % 4]->pt.y +
									cur_quad->corners[(i + 2) % 4]->pt.y) /
								   2;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the same side of the two lines as
						// corner "i". This is given, if the cross product has
						// the same sign for both computations below:
						float a1 = x1 - x2;
						float b1 = y1 - y2;
						// the current corner
						float c11 = cur_quad->corners[i]->pt.x - x2;
						float d11 = cur_quad->corners[i]->pt.y - y2;
						// the candidate corner
						float c12 = quads[k]->corners[j]->pt.x - x2;
						float d12 = quads[k]->corners[j]->pt.y - y2;
						float sign11 = a1 * d11 - c11 * b1;
						float sign12 = a1 * d12 - c12 * b1;

						float a2 = x3 - x4;
						float b2 = y3 - y4;
						// the current corner
						float c21 = cur_quad->corners[i]->pt.x - x4;
						float d21 = cur_quad->corners[i]->pt.y - y4;
						// the candidate corner
						float c22 = quads[k]->corners[j]->pt.x - x4;
						float d22 = quads[k]->corners[j]->pt.y - y4;
						float sign21 = a2 * d21 - c21 * b2;
						float sign22 = a2 * d22 - c22 * b2;

						// Then make shure that two border quads of the same row
						// or
						// column don't link. Check from the current corner's
						// view,
						// whether the corner diagonal from the candidate corner
						// is also on the same side of the two lines as the
						// current
						// corner and the candidate corner.
						float c13 = quads[k]->corners[(j + 2) % 4]->pt.x - x2;
						float d13 = quads[k]->corners[(j + 2) % 4]->pt.y - y2;
						float c23 = quads[k]->corners[(j + 2) % 4]->pt.x - x4;
						float d23 = quads[k]->corners[(j + 2) % 4]->pt.y - y4;
						float sign13 = a1 * d13 - c13 * b1;
						float sign23 = a2 * d23 - c23 * b2;

						// Then check everything from the viewpoint of the
						// candidate quad
						// compute midpoints of "parallel" quad sides 1
						float u1 = (quads[k]->corners[j]->pt.x +
									quads[k]->corners[(j + 1) % 4]->pt.x) /
								   2;
						float v1 = (quads[k]->corners[j]->pt.y +
									quads[k]->corners[(j + 1) % 4]->pt.y) /
								   2;
						float u2 = (quads[k]->corners[(j + 2) % 4]->pt.x +
									quads[k]->corners[(j + 3) % 4]->pt.x) /
								   2;
						float v2 = (quads[k]->corners[(j + 2) % 4]->pt.y +
									quads[k]->corners[(j + 3) % 4]->pt.y) /
								   2;
						// compute midpoints of "parallel" quad sides 2
						float u3 = (quads[k]->corners[j]->pt.x +
									quads[k]->corners[(j + 3) % 4]->pt.x) /
								   2;
						float v3 = (quads[k]->corners[j]->pt.y +
									quads[k]->corners[(j + 3) % 4]->pt.y) /
								   2;
						float u4 = (quads[k]->corners[(j + 1) % 4]->pt.x +
									quads[k]->corners[(j + 2) % 4]->pt.x) /
								   2;
						float v4 = (quads[k]->corners[(j + 1) % 4]->pt.y +
									quads[k]->corners[(j + 2) % 4]->pt.y) /
								   2;

						// MARTIN: Heuristic
						// for the corner "j" of quad "k" to be considered, it
						// needs to be on the same side of the two lines as
						// corner "i". This is again given, if the cross
						// product has the same sign for both computations
						// below:
						float a3 = u1 - u2;
						float b3 = v1 - v2;
						// the current corner
						float c31 = cur_quad->corners[i]->pt.x - u2;
						float d31 = cur_quad->corners[i]->pt.y - v2;
						// the candidate corner
						float c32 = quads[k]->corners[j]->pt.x - u2;
						float d32 = quads[k]->corners[j]->pt.y - v2;
						float sign31 = a3 * d31 - c31 * b3;
						float sign32 = a3 * d32 - c32 * b3;

						float a4 = u3 - u4;
						float b4 = v3 - v4;
						// the current corner
						float c41 = cur_quad->corners[i]->pt.x - u4;
						float d41 = cur_quad->corners[i]->pt.y - v4;
						// the candidate corner
						float c42 = quads[k]->corners[j]->pt.x - u4;
						float d42 = quads[k]->corners[j]->pt.y - v4;
						float sign41 = a4 * d41 - c41 * b4;
						float sign42 = a4 * d42 - c42 * b4;

						// Then make shure that two border quads of the same row
						// or
						// column don't link. Check from the candidate corner's
						// view,
						// whether the corner diagonal from the current corner
						// is also on the same side of the two lines as the
						// current
						// corner and the candidate corner.
						float c33 = cur_quad->corners[(i + 2) % 4]->pt.x - u2;
						float d33 = cur_quad->corners[(i + 2) % 4]->pt.y - v2;
						float c43 = cur_quad->corners[(i + 2) % 4]->pt.x - u4;
						float d43 = cur_quad->corners[(i + 2) % 4]->pt.y - v4;
						float sign33 = a3 * d33 - c33 * b3;
						float sign43 = a4 * d43 - c43 * b4;

						// Check whether conditions are fulfilled
						if (((sign11 < 0 && sign12 < 0) ||
							 (sign11 > 0 && sign12 > 0)) &&
							((sign21 < 0 && sign22 < 0) ||
							 (sign21 > 0 && sign22 > 0)) &&
							((sign31 < 0 && sign32 < 0) ||
							 (sign31 > 0 && sign32 > 0)) &&
							((sign41 < 0 && sign42 < 0) ||
							 (sign41 > 0 && sign42 > 0)) &&
							((sign11 < 0 && sign13 < 0) ||
							 (sign11 > 0 && sign13 > 0)) &&
							((sign21 < 0 && sign23 < 0) ||
							 (sign21 > 0 && sign23 > 0)) &&
							((sign31 < 0 && sign33 < 0) ||
							 (sign31 > 0 && sign33 > 0)) &&
							((sign41 < 0 && sign43 < 0) ||
							 (sign41 > 0 && sign43 > 0)))

						{
							closest_corner_idx = j;
							closest_quad = quads[k];
							min_dist = dist;
						}
					}
				}
			}

			// Have we found a matching corner point?
			if (closest_corner_idx >= 0 && min_dist < FLT_MAX)
			{
				CvCBCorner::Ptr& closest_corner =
					closest_quad->corners[closest_corner_idx];

				// Make shure that the closest quad does not have the current
				// quad as neighbor already
				bool skip = false;
				for (size_t j = 0; !skip && j < 4; j++)
					skip = closest_quad->neighbors[j] == cur_quad;

				if (skip) continue;

				// We've found one more corner - remember it
				closest_corner->pt.x = (pt.x + closest_corner->pt.x) * 0.5f;
				closest_corner->pt.y = (pt.y + closest_corner->pt.y) * 0.5f;

				cur_quad->count++;
				cur_quad->neighbors[i] = closest_quad;
				cur_quad->corners[i] = closest_corner;

				closest_quad->count++;
				closest_quad->neighbors[closest_corner_idx] = cur_quad;
				closest_quad->corners[closest_corner_idx] = closest_corner;
			}
		}
	}
}

//===========================================================================
// AUGMENT PATTERN WITH ADDITIONAL QUADS
//===========================================================================
// The first part of the function is basically a copy of
// "mrFindQuadNeighbors2"
// The comparisons between two points and two lines could be computed in their
// own function
int mrAugmentBestRun(
	std::vector<CvCBQuad::Ptr>& new_quads, int new_dilation,
	std::vector<CvCBQuad::Ptr>& old_quads, int old_dilation)
{
	// thresh dilation is used to counter the effect of dilation on the
	// distance between 2 neighboring corners. Since the distance below is
	// computed as its square, we do here the same. Additionally, we take the
	// conservative assumption that dilation was performed using the 3x3 CROSS
	// kernel, which coresponds to the 4-neighborhood.
	//
	// the "*2" is for the x and y component
	// int idx, i, k, j; // the "3" is for initial corner mismatch
	const float thresh_dilation =
		(float)(2 * new_dilation + 3) * (2 * old_dilation + 3) * 2;
	float dx, dy, dist;

	// Search all old quads which have a neighbor that needs to be linked
	for (size_t idx = 0; idx < old_quads.size(); idx++)
	{
		CvCBQuad::Ptr cur_quad = old_quads[idx];

		// For each corner of this quadrangle
		for (int i = 0; i < 4; i++)
		{
			CvPoint2D32f pt;
			float min_dist = FLT_MAX;
			int closest_corner_idx = -1;
			CvCBQuad::Ptr closest_quad;
			// CvCBCorner *closest_corner = 0;

			// If cur_quad corner[i] is already linked, continue
			if (cur_quad->corners[i]->needsNeighbor == false) continue;

			pt = cur_quad->corners[i]->pt;

			// Look for a match in all new_quads' corners
			for (size_t k = 0; k < new_quads.size(); k++)
			{
				// Only look at unlabeled new quads
				if (new_quads[k]->labeled == true) continue;

				for (int j = 0; j < 4; j++)
				{
					// Only proceed if they are less than dist away from each
					// other
					dx = pt.x - new_quads[k]->corners[j]->pt.x;
					dy = pt.y - new_quads[k]->corners[j]->pt.y;
					dist = dx * dx + dy * dy;

					if ((dist < min_dist) &&
						dist <= (cur_quad->edge_len + thresh_dilation) &&
						dist <= (new_quads[k]->edge_len + thresh_dilation))
					{
						// First Check everything from the viewpoint of the
						// current quad compute midpoints of "parallel" quad
						// sides 1
						float x1 = (cur_quad->corners[i]->pt.x +
									cur_quad->corners[(i + 1) % 4]->pt.x) /
								   2;
						float y1 = (cur_quad->corners[i]->pt.y +
									cur_quad->corners[(i + 1) % 4]->pt.y) /
								   2;
						float x2 = (cur_quad->corners[(i + 2) % 4]->pt.x +
									cur_quad->corners[(i + 3) % 4]->pt.x) /
								   2;
						float y2 = (cur_quad->corners[(i + 2) % 4]->pt.y +
									cur_quad->corners[(i + 3) % 4]->pt.y) /
								   2;
						// compute midpoints of "parallel" quad sides 2
						float x3 = (cur_quad->corners[i]->pt.x +
									cur_quad->corners[(i + 3) % 4]->pt.x) /
								   2;
						float y3 = (cur_quad->corners[i]->pt.y +
									cur_quad->corners[(i + 3) % 4]->pt.y) /
								   2;
						float x4 = (cur_quad->corners[(i + 1) % 4]->pt.x +
									cur_quad->corners[(i + 2) % 4]->pt.x) /
								   2;
						float y4 = (cur_quad->corners[(i + 1) % 4]->pt.y +
									cur_quad->corners[(i + 2) % 4]->pt.y) /
								   2;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the same side of the two lines as
						// corner "i". This is given, if the cross product has
						// the same sign for both computations below:
						float a1 = x1 - x2;
						float b1 = y1 - y2;
						// the current corner
						float c11 = cur_quad->corners[i]->pt.x - x2;
						float d11 = cur_quad->corners[i]->pt.y - y2;
						// the candidate corner
						float c12 = new_quads[k]->corners[j]->pt.x - x2;
						float d12 = new_quads[k]->corners[j]->pt.y - y2;
						float sign11 = a1 * d11 - c11 * b1;
						float sign12 = a1 * d12 - c12 * b1;

						float a2 = x3 - x4;
						float b2 = y3 - y4;
						// the current corner
						float c21 = cur_quad->corners[i]->pt.x - x4;
						float d21 = cur_quad->corners[i]->pt.y - y4;
						// the candidate corner
						float c22 = new_quads[k]->corners[j]->pt.x - x4;
						float d22 = new_quads[k]->corners[j]->pt.y - y4;
						float sign21 = a2 * d21 - c21 * b2;
						float sign22 = a2 * d22 - c22 * b2;

						// Also make shure that two border quads of the same row
						// or
						// column don't link. Check from the current corner's
						// view,
						// whether the corner diagonal from the candidate corner
						// is also on the same side of the two lines as the
						// current
						// corner and the candidate corner.
						float c13 =
							new_quads[k]->corners[(j + 2) % 4]->pt.x - x2;
						float d13 =
							new_quads[k]->corners[(j + 2) % 4]->pt.y - y2;
						float c23 =
							new_quads[k]->corners[(j + 2) % 4]->pt.x - x4;
						float d23 =
							new_quads[k]->corners[(j + 2) % 4]->pt.y - y4;
						float sign13 = a1 * d13 - c13 * b1;
						float sign23 = a2 * d23 - c23 * b2;

						// Second: Then check everything from the viewpoint of
						// the candidate quad. Compute midpoints of "parallel"
						// quad sides 1
						float u1 = (new_quads[k]->corners[j]->pt.x +
									new_quads[k]->corners[(j + 1) % 4]->pt.x) /
								   2;
						float v1 = (new_quads[k]->corners[j]->pt.y +
									new_quads[k]->corners[(j + 1) % 4]->pt.y) /
								   2;
						float u2 = (new_quads[k]->corners[(j + 2) % 4]->pt.x +
									new_quads[k]->corners[(j + 3) % 4]->pt.x) /
								   2;
						float v2 = (new_quads[k]->corners[(j + 2) % 4]->pt.y +
									new_quads[k]->corners[(j + 3) % 4]->pt.y) /
								   2;
						// compute midpoints of "parallel" quad sides 2
						float u3 = (new_quads[k]->corners[j]->pt.x +
									new_quads[k]->corners[(j + 3) % 4]->pt.x) /
								   2;
						float v3 = (new_quads[k]->corners[j]->pt.y +
									new_quads[k]->corners[(j + 3) % 4]->pt.y) /
								   2;
						float u4 = (new_quads[k]->corners[(j + 1) % 4]->pt.x +
									new_quads[k]->corners[(j + 2) % 4]->pt.x) /
								   2;
						float v4 = (new_quads[k]->corners[(j + 1) % 4]->pt.y +
									new_quads[k]->corners[(j + 2) % 4]->pt.y) /
								   2;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the same side of the two lines as
						// corner "i". This is given, if the cross product has
						// the same sign for both computations below:
						float a3 = u1 - u2;
						float b3 = v1 - v2;
						// the current corner
						float c31 = cur_quad->corners[i]->pt.x - u2;
						float d31 = cur_quad->corners[i]->pt.y - v2;
						// the candidate corner
						float c32 = new_quads[k]->corners[j]->pt.x - u2;
						float d32 = new_quads[k]->corners[j]->pt.y - v2;
						float sign31 = a3 * d31 - c31 * b3;
						float sign32 = a3 * d32 - c32 * b3;

						float a4 = u3 - u4;
						float b4 = v3 - v4;
						// the current corner
						float c41 = cur_quad->corners[i]->pt.x - u4;
						float d41 = cur_quad->corners[i]->pt.y - v4;
						// the candidate corner
						float c42 = new_quads[k]->corners[j]->pt.x - u4;
						float d42 = new_quads[k]->corners[j]->pt.y - v4;
						float sign41 = a4 * d41 - c41 * b4;
						float sign42 = a4 * d42 - c42 * b4;

						// Also make shure that two border quads of the same row
						// or
						// column don't link. Check from the candidate corner's
						// view,
						// whether the corner diagonal from the current corner
						// is also on the same side of the two lines as the
						// current
						// corner and the candidate corner.
						float c33 = cur_quad->corners[(i + 2) % 4]->pt.x - u2;
						float d33 = cur_quad->corners[(i + 2) % 4]->pt.y - v2;
						float c43 = cur_quad->corners[(i + 2) % 4]->pt.x - u4;
						float d43 = cur_quad->corners[(i + 2) % 4]->pt.y - v4;
						float sign33 = a3 * d33 - c33 * b3;
						float sign43 = a4 * d43 - c43 * b4;

						// This time we also need to make shure, that no quad
						// is linked to a quad of another dilation run which
						// may lie INSIDE it!!!
						// Third: Therefore check everything from the viewpoint
						// of the current quad compute midpoints of "parallel"
						// quad sides 1
						float x5 = cur_quad->corners[i]->pt.x;
						float y5 = cur_quad->corners[i]->pt.y;
						float x6 = cur_quad->corners[(i + 1) % 4]->pt.x;
						float y6 = cur_quad->corners[(i + 1) % 4]->pt.y;
						// compute midpoints of "parallel" quad sides 2
						float x7 = x5;
						float y7 = y5;
						float x8 = cur_quad->corners[(i + 3) % 4]->pt.x;
						float y8 = cur_quad->corners[(i + 3) % 4]->pt.y;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the other side of the two lines
						// than
						// corner "i". This is given, if the cross product has
						// a different sign for both computations below:
						float a5 = x6 - x5;
						float b5 = y6 - y5;
						// the current corner
						float c51 = cur_quad->corners[(i + 2) % 4]->pt.x - x5;
						float d51 = cur_quad->corners[(i + 2) % 4]->pt.y - y5;
						// the candidate corner
						float c52 = new_quads[k]->corners[j]->pt.x - x5;
						float d52 = new_quads[k]->corners[j]->pt.y - y5;
						float sign51 = a5 * d51 - c51 * b5;
						float sign52 = a5 * d52 - c52 * b5;

						float a6 = x8 - x7;
						float b6 = y8 - y7;
						// the current corner
						float c61 = cur_quad->corners[(i + 2) % 4]->pt.x - x7;
						float d61 = cur_quad->corners[(i + 2) % 4]->pt.y - y7;
						// the candidate corner
						float c62 = new_quads[k]->corners[j]->pt.x - x7;
						float d62 = new_quads[k]->corners[j]->pt.y - y7;
						float sign61 = a6 * d61 - c61 * b6;
						float sign62 = a6 * d62 - c62 * b6;

						// Fourth: Then check everything from the viewpoint of
						// the candidate quad compute midpoints of "parallel"
						// quad sides 1
						float u5 = new_quads[k]->corners[j]->pt.x;
						float v5 = new_quads[k]->corners[j]->pt.y;
						float u6 = new_quads[k]->corners[(j + 1) % 4]->pt.x;
						float v6 = new_quads[k]->corners[(j + 1) % 4]->pt.y;
						// compute midpoints of "parallel" quad sides 2
						float u7 = u5;
						float v7 = v5;
						float u8 = new_quads[k]->corners[(j + 3) % 4]->pt.x;
						float v8 = new_quads[k]->corners[(j + 3) % 4]->pt.y;

						// MARTIN: Heuristic
						// For the corner "j" of quad "k" to be considered,
						// it needs to be on the other side of the two lines
						// than
						// corner "i". This is given, if the cross product has
						// a different sign for both computations below:
						float a7 = u6 - u5;
						float b7 = v6 - v5;
						// the current corner
						float c71 = cur_quad->corners[i]->pt.x - u5;
						float d71 = cur_quad->corners[i]->pt.y - v5;
						// the candidate corner
						float c72 =
							new_quads[k]->corners[(j + 2) % 4]->pt.x - u5;
						float d72 =
							new_quads[k]->corners[(j + 2) % 4]->pt.y - v5;
						float sign71 = a7 * d71 - c71 * b7;
						float sign72 = a7 * d72 - c72 * b7;

						float a8 = u8 - u7;
						float b8 = v8 - v7;
						// the current corner
						float c81 = cur_quad->corners[i]->pt.x - u7;
						float d81 = cur_quad->corners[i]->pt.y - v7;
						// the candidate corner
						float c82 =
							new_quads[k]->corners[(j + 2) % 4]->pt.x - u7;
						float d82 =
							new_quads[k]->corners[(j + 2) % 4]->pt.y - v7;
						float sign81 = a8 * d81 - c81 * b8;
						float sign82 = a8 * d82 - c82 * b8;

						// Check whether conditions are fulfilled
						if (((sign11 < 0 && sign12 < 0) ||
							 (sign11 > 0 && sign12 > 0)) &&
							((sign21 < 0 && sign22 < 0) ||
							 (sign21 > 0 && sign22 > 0)) &&
							((sign31 < 0 && sign32 < 0) ||
							 (sign31 > 0 && sign32 > 0)) &&
							((sign41 < 0 && sign42 < 0) ||
							 (sign41 > 0 && sign42 > 0)) &&
							((sign11 < 0 && sign13 < 0) ||
							 (sign11 > 0 && sign13 > 0)) &&
							((sign21 < 0 && sign23 < 0) ||
							 (sign21 > 0 && sign23 > 0)) &&
							((sign31 < 0 && sign33 < 0) ||
							 (sign31 > 0 && sign33 > 0)) &&
							((sign41 < 0 && sign43 < 0) ||
							 (sign41 > 0 && sign43 > 0)) &&
							((sign51 < 0 && sign52 > 0) ||
							 (sign51 > 0 && sign52 < 0)) &&
							((sign61 < 0 && sign62 > 0) ||
							 (sign61 > 0 && sign62 < 0)) &&
							((sign71 < 0 && sign72 > 0) ||
							 (sign71 > 0 && sign72 < 0)) &&
							((sign81 < 0 && sign82 > 0) ||
							 (sign81 > 0 && sign82 < 0)))
						{
							closest_corner_idx = j;
							closest_quad = new_quads[k];
							min_dist = dist;
						}
					}
				}
			}

			// Have we found a matching corner point?
			if (closest_corner_idx >= 0 && min_dist < FLT_MAX)
			{
				CvCBCorner::Ptr& closest_corner =
					closest_quad->corners[closest_corner_idx];
				closest_corner->pt.x = (pt.x + closest_corner->pt.x) * 0.5f;
				closest_corner->pt.y = (pt.y + closest_corner->pt.y) * 0.5f;

				// We've found one more corner - remember it
				// ATTENTION: write the corner x and y coordinates separately,
				// else the crucial row/column entries will be overwritten !!!
				cur_quad->corners[i]->pt.x = closest_corner->pt.x;
				cur_quad->corners[i]->pt.y = closest_corner->pt.y;
				cur_quad->neighbors[i] = closest_quad;
				closest_quad->corners[closest_corner_idx]->pt.x =
					closest_corner->pt.x;
				closest_quad->corners[closest_corner_idx]->pt.y =
					closest_corner->pt.y;

				// Label closest quad as labeled. In this way we exclude it
				// being considered again during the next loop iteration
				closest_quad->labeled = true;

				// We have a new member of the final pattern, copy it over
				CvCBQuad::Ptr newQuad = CvCBQuad::Ptr(new CvCBQuad);

				newQuad->count = 1;
				newQuad->edge_len = closest_quad->edge_len;
				newQuad->group_idx =
					cur_quad->group_idx;  // the same as the current quad
				newQuad->labeled = false;  // do it right afterwards

				// We only know one neighbor for shure, initialize rest with
				// the nullptr pointer
				newQuad->neighbors[closest_corner_idx] = cur_quad;
				newQuad->neighbors[(closest_corner_idx + 1) % 4]
					.reset();  //	= nullptr;
				newQuad->neighbors[(closest_corner_idx + 2) % 4]
					.reset();  //	= nullptr;
				newQuad->neighbors[(closest_corner_idx + 3) % 4]
					.reset();  //	= nullptr;

				for (int j = 0; j < 4; j++)
				{
					newQuad->corners[j] = CvCBCorner::Ptr(new CvCBCorner);
					newQuad->corners[j]->pt.x = closest_quad->corners[j]->pt.x;
					newQuad->corners[j]->pt.y = closest_quad->corners[j]->pt.y;
				}

				old_quads.push_back(newQuad);
				cur_quad->neighbors[i] = newQuad;

				// Start the function again
				return -1;
			}
		}
	}

	// Finished, don't start the function again
	return 1;
}

//===========================================================================
// GENERATE QUADRANGLES
//===========================================================================
int icvGenerateQuads(
	vector<CvCBQuad::Ptr>& out_quads, vector<CvCBCorner::Ptr>& out_corners,
	const CImage& image, int flags, [[maybe_unused]] int dilation,
	bool firstRun)
{
	// Initializations
	int quad_count = 0;

	// Create temporary storage for contours and the sequence of pointers to
	// found quadrangles
	cv::MemStorage temp_storage = cv::MemStorage(cvCreateMemStorage(0));

	CvSeq* src_contour = nullptr;
	CvSeq* root;  // cv::Seq<> root;  //
	CvContourEx* board = nullptr;
	CvContourScanner scanner;

	// Empiric sower bound for the size of allowable quadrangles.
	// MARTIN, modified: Added "*0.1" in order to find smaller quads.
	const int min_size =
		cvRound(image.getWidth() * image.getHeight() * .03 * 0.01 * 0.92 * 0.1);

	root = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSeq*), temp_storage);

	// Initialize contour retrieving routine
	cv::Mat im_mat = image.asCvMat<cv::Mat>(SHALLOW_COPY);
	IplImage im_ipl = cvIplImage(im_mat);
	scanner = cvStartFindContours(
		&im_ipl, temp_storage, sizeof(CvContourEx), CV_RETR_CCOMP,
		CV_CHAIN_APPROX_SIMPLE);

	// Get all the contours one by one
	while ((src_contour = cvFindNextContour(scanner)) != nullptr)
	{
		CvSeq* dst_contour = nullptr;
		CvRect rect = ((CvContour*)src_contour)->rect;

		// Reject contours with a too small perimeter and contours which are
		// completely surrounded by another contour
		// MARTIN: If this function is called during PART 1, then the parameter
		// "first run"
		// is set to "true". This guarantees, that only "nice" squares are
		// detected.
		// During PART 2, we allow the polygonial approcimation function below
		// to
		// approximate more freely, which can result in recognized "squares"
		// that are in
		// reality multiple blurred and sticked together squares.
		if (CV_IS_SEQ_HOLE(src_contour) && rect.width * rect.height >= min_size)
		{
			int min_approx_level = 2, max_approx_level;
			if (firstRun == true)
				max_approx_level = 3;
			else
				max_approx_level = MAX_CONTOUR_APPROX;
			int approx_level;
			for (approx_level = min_approx_level;
				 approx_level <= max_approx_level; approx_level++)
			{
				dst_contour = cvApproxPoly(
					src_contour, sizeof(CvContour), temp_storage,
					CV_POLY_APPROX_DP, (float)approx_level);

				// We call this again on its own output, because sometimes
				// cvApproxPoly() does not simplify as much as it should.
				dst_contour = cvApproxPoly(
					dst_contour, sizeof(CvContour), temp_storage,
					CV_POLY_APPROX_DP, (float)approx_level);

				if (dst_contour->total == 4) break;
			}

			// Reject non-quadrangles
			if (dst_contour->total == 4 && cvCheckContourConvexity(dst_contour))
			{
				CvPoint pt[4];
				// double d1, d2; //, p = cvContourPerimeter(dst_contour);
				// double area = fabs(cvContourArea(dst_contour, CV_WHOLE_SEQ));
				// double dx, dy;

				for (int i = 0; i < 4; i++)
					pt[i] = *(CvPoint*)cvGetSeqElem(dst_contour, i);

				//                dx = pt[0].x - pt[2].x;
				//                dy = pt[0].y - pt[2].y;
				//                d1 = sqrt(dx*dx + dy*dy);
				//
				//                dx = pt[1].x - pt[3].x;
				//                dy = pt[1].y - pt[3].y;
				//                d2 = sqrt(dx*dx + dy*dy);

				// PHILIPG: Only accept those quadrangles which are more
				// square than rectangular and which are big enough
				//                double d3, d4;
				//                dx = pt[0].x - pt[1].x;
				//                dy = pt[0].y - pt[1].y;
				//                d3 = sqrt(dx*dx + dy*dy);
				//                dx = pt[1].x - pt[2].x;
				//                dy = pt[1].y - pt[2].y;
				//                d4 = sqrt(dx*dx + dy*dy);
				if (true)  //!(flags & CV_CALIB_CB_FILTER_QUADS) ||
				// d3*4 > d4 && d4*4 > d3 && d3*d4 < area*1.5 && area > min_size
				// &&
				// d1 >= 0.15 * p && d2 >= 0.15 * p )
				{
					auto* parent = (CvContourEx*)(src_contour->v_prev);
					parent->counter++;
					if (!board || board->counter < parent->counter)
						board = parent;
					dst_contour->v_prev = (CvSeq*)parent;
					cvSeqPush(root, &dst_contour);
				}
			}
		}
	}

	// Finish contour retrieving
	cvEndFindContours(&scanner);

	// Allocate quad & corner buffers
	out_quads.clear();
	for (int q = 0; q < root->total; q++)
		out_quads.push_back(CvCBQuad::Ptr(new CvCBQuad));

	out_corners.clear();
	for (int q = 0; q < 4 * root->total; q++)
		out_corners.push_back(CvCBCorner::Ptr(new CvCBCorner));

	// Create array of quads structures
	for (int idx = 0; idx < root->total; idx++)
	{
		CvCBQuad::Ptr& q = out_quads[quad_count];
		src_contour = *(CvSeq**)cvGetSeqElem(root, idx);
		if ((flags & cv::CALIB_CB_FILTER_QUADS) &&
			src_contour->v_prev != (CvSeq*)board)
			continue;

		// Reset group ID
		// memset( q, 0, sizeof(*q) );
		q->group_idx = -1;
		assert(src_contour->total == 4);
		for (int i = 0; i < 4; i++)
		{
			CvPoint2D32f pt =
				cvPointTo32f(*(CvPoint*)cvGetSeqElem(src_contour, i));
			CvCBCorner::Ptr& corner = out_corners
				[quad_count * 4 + i];  // &(*out_corners)[quad_count*4
			// + i];

			// memset( corner, 0, sizeof(*corner) );
			corner->pt = pt;
			q->corners[i] = corner;
		}
		q->edge_len = FLT_MAX;
		for (int i = 0; i < 4; i++)
		{
			float dx = q->corners[i]->pt.x - q->corners[(i + 1) & 3]->pt.x;
			float dy = q->corners[i]->pt.y - q->corners[(i + 1) & 3]->pt.y;
			float d = dx * dx + dy * dy;
			if (q->edge_len > d) q->edge_len = d;
		}
		quad_count++;
	}

	if (cvGetErrStatus() < 0)
	{
		out_quads.clear();
		// if( out_corners ) cvFree( out_corners );
		out_corners.clear();
		quad_count = 0;
	}

	cvClearSeq(root);

	return quad_count;
}

// Return 1 on success in finding all the quads, 0 on didn't, -1 on error.
int myQuads2Points(
	const std::vector<CvCBQuad::Ptr>& output_quads, const CvSize& pattern_size,
	std::vector<CvPoint2D32f>& out_corners)
{
	// Initialize
	out_corners.clear();

	bool flagRow = false;
	bool flagColumn = false;
	int maxPattern_sizeRow = -1;
	int maxPattern_sizeColumn = -1;

	// Compute the minimum and maximum row / column ID
	// (it is unlikely that more than 8bit checkers are used per dimension)
	int min_row = 127;
	int max_row = -127;
	int min_column = 127;
	int max_column = -127;

	for (size_t i = 0; i < output_quads.size(); i++)
	{
		const CvCBQuad::Ptr& q = output_quads[i];

		for (int j = 0; j < 4; j++)
		{
			if ((q->corners[j])->row > max_row) max_row = (q->corners[j])->row;
			if ((q->corners[j])->row < min_row) min_row = (q->corners[j])->row;
			if ((q->corners[j])->column > max_column)
				max_column = (q->corners[j])->column;
			if ((q->corners[j])->column < min_column)
				min_column = (q->corners[j])->column;
		}
	}

	// If in a given direction the target pattern size is reached, we know
	// exactly how
	// the checkerboard is oriented.
	// Else we need to prepare enough "dummy" corners for the worst case.
	for (size_t i = 0; i < output_quads.size(); i++)
	{
		const CvCBQuad::Ptr& q = output_quads[i];

		for (int j = 0; j < 4; j++)
		{
			if ((q->corners[j])->column == max_column &&
				(q->corners[j])->row != min_row &&
				(q->corners[j])->row != max_row)
			{
				if ((q->corners[j]->needsNeighbor) == false)
				{
					// We know, that the target pattern size is reached
					// in column direction
					flagColumn = true;
				}
			}
			if ((q->corners[j])->row == max_row &&
				(q->corners[j])->column != min_column &&
				(q->corners[j])->column != max_column)
			{
				if ((q->corners[j]->needsNeighbor) == false)
				{
					// We know, that the target pattern size is reached
					// in row direction
					flagRow = true;
				}
			}
		}
	}

	if (flagColumn == true)
	{
		if (max_column - min_column == pattern_size.width + 1)
		{
			maxPattern_sizeColumn = pattern_size.width;
			maxPattern_sizeRow = pattern_size.height;
		}
		else
		{
			maxPattern_sizeColumn = pattern_size.height;
			maxPattern_sizeRow = pattern_size.width;
		}
	}
	else if (flagRow == true)
	{
		if (max_row - min_row == pattern_size.width + 1)
		{
			maxPattern_sizeRow = pattern_size.width;
			maxPattern_sizeColumn = pattern_size.height;
		}
		else
		{
			maxPattern_sizeRow = pattern_size.height;
			maxPattern_sizeColumn = pattern_size.width;
		}
	}
	else
	{
		// If target pattern size is not reached in at least one of the two
		// directions,  then we do not know where the remaining corners are
		// located. Account for this.
		maxPattern_sizeColumn = max(pattern_size.width, pattern_size.height);
		maxPattern_sizeRow = max(pattern_size.width, pattern_size.height);
	}

	// JL: Check sizes:
	if (maxPattern_sizeRow * maxPattern_sizeColumn !=
		pattern_size.width * pattern_size.height)
		return 0;  // Bad...
	// and also, swap rows/columns so we always have consistently the points in
	// the order: first all columns in a row, then the next row, etc...
	bool do_swap_col_row = maxPattern_sizeRow != pattern_size.height;

	if (do_swap_col_row)
	{
		std::swap(min_row, min_column);
		std::swap(maxPattern_sizeRow, maxPattern_sizeColumn);
	}

	// Write the corners in increasing order to "out_corners"

	for (int i = min_row + 1; i < maxPattern_sizeRow + min_row + 1; i++)
	{
		for (int j = min_column + 1; j < maxPattern_sizeColumn + min_column + 1;
			 j++)
		{
			// Reset the iterator
			int iter = 1;

			for (size_t k = 0; k < output_quads.size(); k++)
			{
				for (int l = 0; l < 4; l++)
				{
					int r = output_quads[k]->corners[l]->row;
					int c = output_quads[k]->corners[l]->column;
					if (do_swap_col_row) std::swap(r, c);

					if (r == i && c == j)
					{
						// Only write corners to the output file, which are
						// connected
						// i.e. only if iter == 2
						if (iter == 2)
						{
							// The respective row and column have been found,
							// save point:
							out_corners.push_back(
								output_quads[k]->corners[l]->pt);
						}

						// If the iterator is larger than two, this means that
						// more than
						// two corners have the same row / column entries. Then
						// some
						// linking errors must have occured and we should not
						// use the found
						// pattern
						if (iter > 2) return -1;

						iter++;
					}
				}
			}

			// If the respective row / column is non - existent or is a border
			// corner
			if (iter == 1 || iter == 2)
			{
				// cornersX << -1;
				// cornersX << " ";
				// cornersY << -1;
				// cornersY << " ";
			}
		}
	}

	// All corners found?
	return (out_corners.size() ==
			size_t(pattern_size.width) * size_t(pattern_size.height))
			   ? 1
			   : 0;
}

// Make unique all the (smart pointers-pointed) objects in the list and
// neighbors lists.
void quadListMakeUnique(std::vector<CvCBQuad::Ptr>& quads)
{
	std::map<CvCBQuad*, size_t> pointer2index;
	for (size_t i = 0; i < quads.size(); i++) pointer2index[quads[i].get()] = i;

	vector<std::array<size_t, 4>> neig_indices(quads.size());
	for (size_t i = 0; i < quads.size(); i++)
		for (size_t j = 0; j < 4; j++)
			neig_indices[i][j] = pointer2index[quads[i]->neighbors[j].get()];

	std::vector<CvCBQuad::Ptr> new_quads = quads;
	std::for_each(new_quads.begin(), new_quads.end(), [](CvCBQuad::Ptr& ptr) {
		ptr = CvCBQuad::Ptr(new CvCBQuad(*ptr));
	});
	for (size_t i = 0; i < new_quads.size(); i++)
		for (size_t j = 0; j < 4; j++)
			new_quads[i]->neighbors[j] = new_quads[neig_indices[i][j]];
}

//===========================================================================
// END OF FILE  (Of "OCamCalib Toolbox" code)
//===========================================================================

#endif  // MRPT_HAS_OPENCV
