/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "vision-precomp.h"   // Precompiled headers

#include <mrpt/math/kmeans.h>
#include <mrpt/math/geometry.h>
#include <list>

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h>
#include "checkerboard_ocamcalib_detector.h"

#if VIS
#include <mrpt/gui/CDisplayWindow.h>
#endif


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace std;


#if MRPT_HAS_OPENCV

// Return: true: found OK
bool find_chessboard_corners_multiple(
	const mrpt::utils::CImage & img_,
	CvSize pattern_size,
	std::vector< std::vector<CvPoint2D32f> > &out_corners)
{
	// Assure it's a grayscale image:
	const mrpt::utils::CImage img( img_, FAST_REF_OR_CONVERT_TO_GRAY );

    mrpt::utils::CImage thresh_img(img.getWidth(),img.getHeight(), CH_GRAY );
	mrpt::utils::CImage thresh_img_save(img.getWidth(),img.getHeight(), CH_GRAY );

	out_corners.clear();  // for now, empty the output.

    const size_t expected_quads_count = ((pattern_size.width + 1)*(pattern_size.height + 1) + 1)/2;

	// PART 0: INITIALIZATION
	//-----------------------------------------------------------------------
	// Initialize variables
	int flags					=  1;	// not part of the function call anymore!
    //int found					=  0;

	vector<CvCBQuadPtr>		quads;
    vector<CvCBCornerPtr>	corners;
	list< vector<CvCBQuadPtr> > good_quad_groups;  // Storage of potential good quad groups found

    if( pattern_size.width < 2 || pattern_size.height < 2 )
	{
        std::cerr  << "Pattern should have at least 2x2 size" << endl;
		return false;
	}
	if( pattern_size.width > 127 || pattern_size.height > 127 )
	{
        std::cerr  << "Pattern should not have a size larger than 127 x 127" << endl;
		return false;
	}

	// JL: Move these constructors out of the loops:
	IplConvKernel *kernel_cross = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CROSS,NULL);
	IplConvKernel *kernel_rect = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);

	static int kernel_diag1_vals[9] = {
		1,0,0,
		0,1,0,
		0,0,1 };
	IplConvKernel *kernel_diag1 = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CUSTOM,kernel_diag1_vals);
	static int kernel_diag2_vals[9] = {
		0,0,1,
		0,1,0,
		1,0,0 };
	IplConvKernel *kernel_diag2 = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CUSTOM,kernel_diag2_vals);
	static int kernel_horz_vals[9] = {
		0,0,0,
		1,1,1,
		0,0,0 };
	IplConvKernel *kernel_horz = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CUSTOM,kernel_horz_vals);
	static int kernel_vert_vals[9] = {
		0,1,0,
		0,1,0,
		0,1,0 };
	IplConvKernel *kernel_vert = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_CUSTOM,kernel_vert_vals);

	// For image binarization (thresholding)
    // we use an adaptive threshold with a gaussian mask
	// ATTENTION: Gaussian thresholding takes MUCH more time than Mean thresholding!
    int block_size = cvRound(MIN(img.getWidth(),img.getHeight())*0.2)|1;

	cvAdaptiveThreshold( img.getAs<IplImage>(), thresh_img.getAs<IplImage>(), 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, block_size, 0 );

	cvCopy( thresh_img.getAs<IplImage>(), thresh_img_save.getAs<IplImage>());

	// PART 1: FIND LARGEST PATTERN
	//-----------------------------------------------------------------------
	// Checker patterns are tried to be found by dilating the background and
	// then applying a canny edge finder on the closed contours (checkers).
	// Try one dilation run, but if the pattern is not found, repeat until
	// max_dilations is reached.
    //for( int dilations = min_dilations; dilations <= max_dilations; dilations++ )

	bool last_dilation = false;

	for( int dilations = 0; !last_dilation; dilations++ )
    {
		// Calling "cvCopy" again is much faster than rerunning "cvAdaptiveThreshold"
		cvCopy( thresh_img_save.getAs<IplImage>(), thresh_img.getAs<IplImage>() );

		// Dilate squares:
		last_dilation = do_special_dilation(thresh_img, dilations,kernel_cross,kernel_rect,kernel_diag1,kernel_diag2, kernel_horz,kernel_vert);

        // In order to find rectangles that go to the edge, we draw a white
		// line around the image edge. Otherwise FindContours will miss those
		// clipped rectangle contours. The border color will be the image mean,
		// because otherwise we risk screwing up filters like cvSmooth()
        cvRectangle( thresh_img.getAs<IplImage>(), cvPoint(0,0),
					cvPoint(thresh_img.getWidth()-1,thresh_img.getHeight()-1),
					CV_RGB(255,255,255), 3, 8);

		// Generate quadrangles in the following function
		// "quad_count" is the number of cound quadrangles
        int quad_count = icvGenerateQuads( quads, corners, thresh_img, flags, dilations, true );
        if( quad_count <= 0 )
            continue;

		// The following function finds and assigns neighbor quads to every
		// quadrangle in the immediate vicinity fulfilling certain
		// prerequisites
		mrFindQuadNeighbors2( quads, dilations);

		// JL: To achieve multiple-checkerboard, take all the raw detected quads and
		//  separate them in groups with k-means.
		vector<CArrayDouble<2>, Eigen::aligned_allocator<CArrayDouble<2> > > quad_centers;
		quad_centers.resize(quads.size());
		for (size_t i=0;i<quads.size();i++)
		{
			const CvCBQuad* q= quads[i].pointer();
			quad_centers[i][0] = 0.25 * (q->corners[0]->pt.x + q->corners[1]->pt.x + q->corners[2]->pt.x + q->corners[3]->pt.x);
			quad_centers[i][1] = 0.25 * (q->corners[0]->pt.y + q->corners[1]->pt.y + q->corners[2]->pt.y + q->corners[3]->pt.y);
		}

		// Try the k-means with a number of variable # of clusters:
		static const size_t MAX_NUM_CLUSTERS = 4;
		for (size_t nClusters=1;nClusters<MAX_NUM_CLUSTERS;nClusters++)
		{
			vector<size_t> num_quads_by_cluster(nClusters);

			vector<int>	assignments;
			mrpt::math::kmeanspp<
				vector<CArrayDouble<2>,Eigen::aligned_allocator<CArrayDouble<2> > >,
				vector<CArrayDouble<2>,Eigen::aligned_allocator<CArrayDouble<2> > > >
				(nClusters,quad_centers,assignments);

			// Count # of quads in each cluster:
			for (size_t i=0;i<nClusters;i++)
				num_quads_by_cluster[i] = std::count(assignments.begin(),assignments.end(), i);

#if VIS
			{
				static mrpt::gui::CDisplayWindow  win;
				win.setWindowTitle( format("All quads (%u) | %u clusters",
					(unsigned)quad_centers.size(),
					(unsigned)nClusters ));
				mrpt::utils::CImage im;
				img.colorImage(im);
				for (size_t i=0;i<quad_centers.size();i++)
				{
					static const TColor colors[4] = { TColor(255,0,0),TColor(0,0,255),TColor(255,0,255),TColor(0,255,0) };
					im.cross(quad_centers[i][0],quad_centers[i][1], colors[assignments[i]%4],'+', 10);
				}
				win.showImage(im);
				win.waitForKey();
			}
#endif // VIS


			// Take a look at the promising clusters:
			// -----------------------------------------
			for (size_t i=0;i<nClusters;i++)
			{
				if (num_quads_by_cluster[i]<size_t(pattern_size.height*pattern_size.width))
					continue; // Can't be good...

				// Create a subset of the quads with those in the i'th cluster:
				vector<CvCBQuadPtr> ith_quads;
				for (size_t q=0;q<quads.size();q++)
					if (size_t(assignments[q])==i)
						ith_quads.push_back(quads[q]);

				// Make sense out of smart pointers...
				quadListMakeUnique(ith_quads);

				// The connected quads will be organized in groups. The following loop
				// increases a "group_idx" identifier.
				// The function "icvFindConnectedQuads assigns all connected quads
				// a unique group ID.
				// If more quadrangles were assigned to a given group (i.e. connected)
				// than are expected by the input variable "pattern_size", the
				// function "icvCleanFoundConnectedQuads" erases the surplus
				// quadrangles by minimizing the convex hull of the remaining pattern.
				for( int group_idx = 0; ; group_idx++ )
				{
					vector<CvCBQuadPtr>		quad_group;

					icvFindConnectedQuads( ith_quads , quad_group, group_idx, dilations );
					if( quad_group.empty() )
						break;

					icvCleanFoundConnectedQuads( quad_group, pattern_size );
					size_t count = quad_group.size();

					if( count == expected_quads_count )
					{
#if VIS
						{
							static mrpt::gui::CDisplayWindow  win;
							win.setWindowTitle( format("Candidate group #%i (%i)",(int)group_idx, (int)quad_group.size()));
							mrpt::utils::CImage im;
							img.colorImage(im);
							for (size_t i=0;i<quad_group.size();i++)
							{
								static const TColor colors[4] = { TColor(255,0,0),TColor(0,0,255),TColor(255,0,255),TColor(0,255,0) };
								const double x=0.25*(quad_group[i]->corners[0]->pt.x+quad_group[i]->corners[1]->pt.x+quad_group[i]->corners[2]->pt.x+quad_group[i]->corners[3]->pt.x);
								const double y=0.25*(quad_group[i]->corners[0]->pt.y+quad_group[i]->corners[1]->pt.y+quad_group[i]->corners[2]->pt.y+quad_group[i]->corners[3]->pt.y);
								im.cross(x,y, colors[group_idx%4],'+', 10);
							}
							win.showImage(im);
							win.waitForKey();
						}
#endif // VIS

						// The following function labels all corners of every quad
						// with a row and column entry.
						mrLabelQuadGroup( quad_group, pattern_size, true );

						// Add this set of quads as a good result to be returned to the user:
						good_quad_groups.push_back(quad_group);
						// And "make_unique()" it:
						//quadListMakeUnique( good_quad_groups.back() );
					}

				} // end for each "group_idx"

			} // end of the loop "try with each promising cluster"

		} // end loop, for each "nClusters" size.

	} // end for dilation


	// Convert the set of good detected quad sets in "good_quad_groups"
	//  to the expected output data struct, doing a final check to
	//  remove duplicates:
	vector<TPoint2D> out_boards_centers; // the center (average) of each output board.
	for (list<vector<CvCBQuadPtr> >::const_iterator it=good_quad_groups.begin();it!=good_quad_groups.end();++it)
	{
		// Compute the center of this board:
		TPoint2D boardCenter(0,0);
		for (size_t i=0;i<it->size();i++)
		{	// JL: Avoid the normalizations of all the averages, since it really doesn't matter for our purpose.
			boardCenter += TPoint2D(
				/*0.25* */ (*it)[i]->corners[0]->pt.x+(*it)[i]->corners[1]->pt.x+(*it)[i]->corners[2]->pt.x+(*it)[i]->corners[3]->pt.x,
				/*0.25* */ (*it)[i]->corners[0]->pt.y+(*it)[i]->corners[1]->pt.y+(*it)[i]->corners[2]->pt.y+(*it)[i]->corners[3]->pt.y );
		}

		// If it's too close to an already existing board, it's surely a duplicate:
		double min_dist = std::numeric_limits<double>::max();
		for (size_t b=0;b<out_boards_centers.size();b++)
			keep_min(min_dist, mrpt::math::distance(boardCenter, out_boards_centers[b]) );

		if (out_corners.empty() || min_dist > 80 )
		{
			vector<CvPoint2D32f> pts;
			if (1==myQuads2Points( *it, pattern_size,pts ))	// and populate it now.
			{
				// Ok with it: add to the output list:
				out_corners.push_back(pts);
				// Add center to list of known centers:
				out_boards_centers.push_back(boardCenter);
			}
		}
	}


	// Free mem:
	cvReleaseStructuringElement(&kernel_cross);
	cvReleaseStructuringElement(&kernel_rect);
	cvReleaseStructuringElement(&kernel_diag1);
	cvReleaseStructuringElement(&kernel_diag2);
	cvReleaseStructuringElement(&kernel_horz);
	cvReleaseStructuringElement(&kernel_vert);

	return !out_corners.empty();
}


#endif // MRPT_HAS_OPENCV

