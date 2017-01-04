/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "sift.h"
#include "imgfeatures.h"
#include "utils.h"

//#include "cxcore.h"
//#include <highgui.h>
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#include <stdio.h>

/******************************** Globals ************************************/

char* img_file_name = "..\\beaver.png";
char* out_file_name  = "..\\beaver.sift";;
char* out_img_name = NULL;
int display = 1;
int intvls = SIFT_INTVLS;
double sigma = SIFT_SIGMA;
double contr_thr = SIFT_CONTR_THR;
int curv_thr = SIFT_CURV_THR;
int img_dbl = SIFT_IMG_DBL;
int descr_width = SIFT_DESCR_WIDTH;
int descr_hist_bins = SIFT_DESCR_HIST_BINS;


/********************************** Main *************************************/

int main( int argc, char** argv )
{
	IplImage* img;
	struct feature* features;
	int n = 0;

	fprintf( stderr, "Finding SIFT features...\n" );
	img = cvLoadImage( img_file_name, 1 );
	if( ! img )
	{
		fprintf( stderr, "unable to load image from %s", img_file_name );
		exit( 1 );
	}
	n = _sift_features( img, &features, intvls, sigma, contr_thr, curv_thr,
						img_dbl, descr_width, descr_hist_bins );
	fprintf( stderr, "Found %d features.\n", n );

	if( display )
	{
		draw_features( img, features, n );
		cvNamedWindow( img_file_name, 1 );
		cvShowImage( img_file_name, img );
		cvWaitKey( 0 );
	}

	if( out_file_name != NULL )
		export_features( out_file_name, features, n );

	if( out_img_name != NULL )
		cvSaveImage( out_img_name, img, NULL );
	return 0;
}
