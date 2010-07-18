/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
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

#include <mrpt/math/kmeans.h>

#include "checkerboard_ocamcalib_detector.h"


using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;

#if MRPT_HAS_OPENCV

// Return: true: found OK
bool find_chessboard_corners_multiple( 
	const mrpt::utils::CImage & img_, 
	CvSize pattern_size, 
	std::vector< std::vector<CvPoint2D32f> > &out_corners)
{
	// Assure it's a grayscale image:
	CImage img(UNINITIALIZED_IMAGE);
	if (img_.isColor())
			img_.grayscale(img);
	else 	img.setFromImageReadOnly(img_);

    CImage thresh_img(img.getWidth(),img.getHeight(), CH_GRAY );
	CImage thresh_img_save(img.getWidth(),img.getHeight(), CH_GRAY );


	// PART 0: INITIALIZATION
	//-----------------------------------------------------------------------
	// Initialize variables
	int flags					=  1;	// not part of the function call anymore!
	size_t max_count				=  0;
	int max_dilation_run_ID		= -1;
    int found					=  0;

	vector<CvCBQuadPtr>		quads;			// CvCBQuad **quads = 0;
	vector<CvCBQuadPtr>		quad_group;		// CvCBQuad **quad_group		=  0;
    vector<CvCBCornerPtr>	corners;		// CvCBCorner *corners			=  0;
	vector<CvCBQuadPtr>		output_quad_group;	//	CvCBQuad **output_quad_group = 0;

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

		// JL: To achieve multiple-checkerboard, take all the raw detected quads and 
		//  separate them in groups with k-means.
		vector<CArrayDouble<2> >	quad_centers;
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
			const double final_cost = mrpt::math::kmeanspp< vector<CArrayDouble<2> >, vector<CArrayDouble<2> > >(
				nClusters,quad_centers,assignments);

			// Count # of quads in each cluster:
			for (size_t i=0;i<nClusters;i++)
				num_quads_by_cluster[i] = std::count(assignments.begin(),assignments.end(), i);

#if VIS
			{
				static mrpt::gui::CDisplayWindow  win;
				win.setWindowTitle( format("All quads (%u) | %u clusters",
					(unsigned)quad_centers.size(),
					(unsigned)nClusters ));
				CImage im;
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
					{
						ith_quads.push_back(quads[i]);
						ith_quads.back().make_unique(); // make unique so it's decouppled of the main original list.
					}

				// The following function finds and assigns neighbor quads to every
				// quadrangle in the immediate vicinity fulfilling certain
				// prerequisites
				mrFindQuadNeighbors2( ith_quads, quad_count, dilations);

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
					icvFindConnectedQuads( ith_quads, quad_group, group_idx, dilations );
					if( quad_group.empty() )
						break;

					icvCleanFoundConnectedQuads( quad_group, pattern_size );
					size_t count = quad_group.size();

#if VIS
			{
				static mrpt::gui::CDisplayWindow  win;
				win.setWindowTitle( format("Candidate group (%i)", count));
				CImage im;
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

					// MARTIN's Code
					// To save computational time, only proceed, if the number of
					// found quads during this dilation run is larger than the
					// largest previous found number
					if( count /*>=*/ >  max_count)
					{
						cout << "CHECKERBOARD: Best found at dilation=" << dilations << endl;

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
						mrLabelQuadGroup( quad_group, max_count, pattern_size, true );

						// Allocate memory
						//output_quad_group.resize( (pattern_size.height+2) * (pattern_size.width+2) ); // = (CvCBQuad**)cvAlloc( sizeof(output_quad_group[0]) * ((pattern_size.height+2) * (pattern_size.width+2)) );
						// The following function copies every member of "quad_group"
						// to "output_quad_group", because "quad_group" will be
						// overwritten during the next loop pass.
						// "output_quad_group" is a true copy of "quad_group" and
						// later used for output
						output_quad_group = quad_group; // mrCopyQuadGroup( quad_group, output_quad_group, max_count );
					}

				} // end for each "group_idx"

			} // end of the loop "try with each promising cluster"

		} // end loop, for each "nClusters" size.

	} // end for dilation

	// Convert to the expected output data struct:
	//myQuads2Points( output_quad_group, pattern_size,out_corners);

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

