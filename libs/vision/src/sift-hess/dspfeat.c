/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "imgfeatures.h"
#include "utils.h"

//#include "cxcore.h"
//#include <highgui.h>
// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#include <stdio.h>

/******************************** Globals ************************************/

char* feat_file = "..\\beaver.sift";
char* img_file = "..\\beaver.png";
int feat_type = FEATURE_LOWE;

/********************************** Main *************************************/


int main( int argc, char** argv )
{
	IplImage* img;
	struct feature* feat;
	char* name;
	int n;

	img = cvLoadImage( img_file, 1 );
	if( ! img )
		fatal_error( "unable to load image from %s", img_file );
	n = import_features( feat_file, feat_type, &feat );
	if( n == -1 )
		fatal_error( "unable to import features from %s", feat_file );
	name = feat_file;

	draw_features( img, feat, n );
	cvNamedWindow( name, 1 );
	cvShowImage( name, img );
	cvWaitKey( 0 );
	return 0;
}
