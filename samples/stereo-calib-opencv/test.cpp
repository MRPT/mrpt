/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
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

/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warrenty, support or any guarentee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008

   AVAILABLE AT:
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

   OTHER OPENCV SITES:
   * The source code is on sourceforge at:
     http://sourceforge.net/projects/opencvlibrary/
   * The OpenCV wiki page (As of Oct 1, 2008 this is down for changing over servers, but should come back):
     http://opencvlibrary.sourceforge.net/
   * An active user group is at:
     http://tech.groups.yahoo.com/group/OpenCV/
   * The minutes of weekly OpenCV development meetings are at:
     http://pr.willowgarage.com/wiki/OpenCV
   ************************************************** */

#include <mrpt/config.h>

#include <mrpt/otherlibs/do_opencv_includes.h> 

#include <vector>
#include <string>
#include <algorithm>

#include <stdio.h>

#if MRPT_HAS_OPENCV
#include <ctype.h>

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/CMatrixFixedNumeric.h>

using namespace std;

// Auxiliary classes
#include <ios>
#include <istream>
#include <limits>

template <typename CharT>
std::streamsize ignore_line (
  std::basic_istream<CharT>& in, bool always_discard = false )
{
  std::streamsize nread = 0;

  if ( always_discard
    || ( in.rdbuf()->sungetc() != std::char_traits<CharT>::eof()
    && in.get() != in.widen ( '\n' ) ) )
  {
    // The stream is good, and we haven't
    // read a full line yet, so clear it out
    in.ignore ( std::numeric_limits<std::streamsize>::max(), in.widen ( '\n' ) );
    nread = in.gcount();
  }
  return nread;
}
class ignoreline {
  bool _always_discard;
  mutable std::streamsize _nread;
public:
  ignoreline ( bool always_discard = false )
	: _always_discard ( always_discard ), _nread ( 0 )
  {}

  std::streamsize gcount() const { return _nread; }

  template <typename CharT>
  friend std::basic_istream<CharT>& operator>> (
	std::basic_istream<CharT>& in, const ignoreline& manip )
  {
	manip._nread = ignore_line ( in, manip._always_discard );
	return in;
  }
};

bool askBooleanQuestion(const char*msg, const bool default_val)
{
	string sBool;
	cin >> ignoreline(); // clear cin buffer... :-(
	cout << msg;
	std::getline(cin,sBool);
	if (sBool.empty()) return default_val;
	else return !(sBool[0]=='N' || sBool[0]=='n');
}



//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag: useCalibrated for calibrated (0) or
// uncalibrated (1: use cvStereoCalibrate(), 2: compute fundamental
// matrix separately) stereo. Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//
void StereoCalib(
	const char* imageList,
	const char* sOutFile,
	int nx, int ny,
	int useUncalibrated,
	const float squareSize,
	const double alpha,
	const bool flag_fix_aspect_ratio,
	const bool flag_zero_tangent_dist,
	const bool flag_same_focal_len
	)
{
    int displayCorners = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
                                      //or up-down camera arrangements
    const int maxScale = 1;

    FILE* f = fopen(imageList, "rt");

    int i, j, lr, nframes, n = nx*ny, N = 0;
    vector<string> imageNames[2];
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {0,0};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    if( displayCorners )
        cvNamedWindow( "corners", 1 );

    // READ IN THE LIST OF CHESSBOARDS:
    if( !f )
    {
        fprintf(stderr, "can not open file %s\n", imageList );
        return;
    }

	std::vector<std::string> lst_image_files;


    printf("Starting main loop\n");
    for(i=0;;i++)
    {
        printf("Iteration %d\n", i);
        char buf[1024];
        int count = 0, result=0;
        lr = i % 2;
        vector<CvPoint2D32f>& pts = points[lr];
        if( !fgets( buf, sizeof(buf)-3, f ))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        IplImage* img = cvLoadImage( buf, 0 );
        if( !img )
            break;

		lst_image_files.push_back(string(buf));

        imageSize = cvGetSize(img);
        imageNames[lr].push_back(buf);
    //FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                    img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(nx, ny),
                &temp[0], &count,
                CV_CALIB_CB_ADAPTIVE_THRESH |
                CV_CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
            {
                temp[j].x /= s;
                temp[j].y /= s;
            }
            if( result )
                break;
        }
        if( displayCorners )
        {
            printf("%s\n", buf);
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
                count, result );
            cvShowImage( "corners", cimg );
            cvReleaseImage( &cimg );
            int c = cvWaitKey(100);
            if( c == 27 || c == 'q' || c == 'Q' ) //Allow ESC to quit
                exit(-1);
        }
        else
            putchar('.');
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        active[lr].push_back((uchar)result);
    //assert( result != 0 );
        if( result )
        {
         //Calibration will suffer without subpixel interpolation
            cvFindCornerSubPix( img, &temp[0], count,
                cvSize(11, 11), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                30, 0.01) );
            copy( temp.begin(), temp.end(), pts.begin() + N );
        }
        cvReleaseImage( &img );
    }
    fclose(f);
    printf("\n");
// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    nframes = active[0].size();//Number of good chessboads found
    objectPoints.resize(nframes*n);
    for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
        objectPoints[i*nx + j] =
        cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + n,
        objectPoints.begin() + i*n );
    npoints.resize(nframes,n);
    N = nframes*n;
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1);
    cvSetIdentity(&_M2);
    cvZero(&_D1);
    cvZero(&_D2);

	// CALIBRATE THE STEREO CAMERAS
	// ======================================================
    printf("Running stereo calibration ...");
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1,
        &_imagePoints2, &_npoints,
        &_M1, &_D1, &_M2, &_D2,
        imageSize, &_R, &_T, &_E, &_F,
        cvTermCriteria(CV_TERMCRIT_ITER+
        CV_TERMCRIT_EPS, 150, 1e-6),
		(flag_fix_aspect_ratio ? CV_CALIB_FIX_ASPECT_RATIO:0)
		+
		(flag_zero_tangent_dist ? CV_CALIB_ZERO_TANGENT_DIST:0)
		+
		(flag_same_focal_len ? CV_CALIB_SAME_FOCAL_LENGTH:0) );
    printf(" done\n");


	// CALIBRATION QUALITY CHECK
	// ======================================================
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
//Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
        &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
        &_M2, &_D2, 0, &_M2 );
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
            points[0][i].y*lines[1][i].y + lines[1][i].z)
            + fabs(points[1][i].x*lines[0][i].x +
            points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(nframes*n) );



	//COMPUTE RECTIFICATION
	// ========================================================
    CvMat* mx1 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* my1 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* mx2 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* my2 = cvCreateMat( imageSize.height,
        imageSize.width, CV_32F );
    CvMat* img1r = cvCreateMat( imageSize.height,
        imageSize.width, CV_8U );
    CvMat* img2r = cvCreateMat( imageSize.height,
        imageSize.width, CV_8U );
    CvMat* disp = cvCreateMat( imageSize.height,
        imageSize.width, CV_16S );
    CvMat* vdisp = cvCreateMat( imageSize.height,
        imageSize.width, CV_8U );
    CvMat* pair;
    double R1[3][3], R2[3][3], P1[3][4], P2[3][4], Q[4][4];
    CvMat _R1 = cvMat(3, 3, CV_64F, R1);
    CvMat _R2 = cvMat(3, 3, CV_64F, R2);
    CvMat _Q = cvMat(4, 4, CV_64F, Q);
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useUncalibrated == 0 )
    {
        CvMat _P1 = cvMat(3, 4, CV_64F, P1);
        CvMat _P2 = cvMat(3, 4, CV_64F, P2);
#if MRPT_OPENCV_VERSION_NUM<0x210
	// OpenCV 2.0.X
	cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
		&_R, &_T,
		&_R1, &_R2, &_P1, &_P2, &_Q,
		0/*CV_CALIB_ZERO_DISPARITY*/
		);
#else
	// OpenCV 2.1.X - 2.2.X - 2.3.X
	cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
		&_R, &_T,
		&_R1, &_R2, &_P1, &_P2, &_Q,
		0 /* CV_CALIB_ZERO_DISPARITY */,
		0 /* alpha */
		);
#endif
		isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
//Precompute maps for cvRemap()
        cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
        cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
    }
//OR ELSE HARTLEY'S METHOD
    else if( useUncalibrated == 1 || useUncalibrated == 2 )
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        double H1[3][3], H2[3][3], iM[3][3];
        CvMat _H1 = cvMat(3, 3, CV_64F, H1);
        CvMat _H2 = cvMat(3, 3, CV_64F, H2);
        CvMat _iM = cvMat(3, 3, CV_64F, iM);
//Just to show you could have independently used F
        if( useUncalibrated == 2 )
            cvFindFundamentalMat( &_imagePoints1,
            &_imagePoints2, &_F);
        cvStereoRectifyUncalibrated( &_imagePoints1,
            &_imagePoints2, &_F,
            imageSize,
            &_H1, &_H2, 3);
        cvInvert(&_M1, &_iM);
        cvMatMul(&_H1, &_M1, &_R1);
        cvMatMul(&_iM, &_R1, &_R1);
        cvInvert(&_M2, &_iM);
        cvMatMul(&_H2, &_M2, &_R2);
        cvMatMul(&_iM, &_R2, &_R2);
//Precompute map for cvRemap()
        cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);

        cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
    }
    else
        assert(0);



	// SAVE CALIBRATION REPORT FILE
	// ==============================================================================
	cout << "Writing report file: " << sOutFile<< endl;
	FILE *f_out = fopen(sOutFile,"wt");

	time_t systime;
	time(&systime);
	struct tm * timeinfo=localtime(&systime);

	fprintf( f_out,
		"# Stereo camera calibration report\n"
		"# Generated by camera-calib-gui - MRPT at %s"
		"# (This file is loadable from rawlog-edit and other MRPT tools)\n"
		"# ---------------------------------------------------------------------\n\n",
		asctime(timeinfo)
		);

	fprintf( f_out,
		"# Left camera calibration parameters:\n"
		"[CAMERA_PARAMS_LEFT]\n"
		"resolution = [%u %u]\n"
		"cx         = %f\n"
		"cy         = %f\n"
		"fx         = %f\n"
		"fy         = %f\n"
		"dist       = [%e %e %e %e %e]    // The order is: [K1 K2 T1 T2 K3]\n\n",
			imageSize.width,imageSize.height,
			cvGet2D(&_M1,0,2).val[0],
			cvGet2D(&_M1,1,2).val[0],
			cvGet2D(&_M1,0,0).val[0],
			cvGet2D(&_M1,1,1).val[0],
			cvGet2D(&_D1,0,0).val[0], cvGet2D(&_D1,0,1).val[0], cvGet2D(&_D1,0,2).val[0],
			cvGet2D(&_D1,0,3).val[0], cvGet2D(&_D1,0,4).val[0] );

	fprintf( f_out,
		"# Right camera calibration parameters:\n"
		"[CAMERA_PARAMS_RIGHT]\n"
		"resolution = [%u %u]\n"
		"cx         = %f\n"
		"cy         = %f\n"
		"fx         = %f\n"
		"fy         = %f\n"
		"dist       = [%e %e %e %e %e]    // The order is: [K1 K2 T1 T2 K3]\n\n",
			imageSize.width,imageSize.height,
			cvGet2D(&_M2,0,2).val[0],
			cvGet2D(&_M2,1,2).val[0],
			cvGet2D(&_M2,0,0).val[0],
			cvGet2D(&_M2,1,1).val[0],
			cvGet2D(&_D2,0,0).val[0], cvGet2D(&_D2,0,1).val[0], cvGet2D(&_D2,0,2).val[0],
			cvGet2D(&_D2,0,3).val[0], cvGet2D(&_D2,0,4).val[0] );


	// Convert RT to MRPT classes:
	mrpt::math::CMatrixFixedNumeric<double,3,3> mROT;
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
			mROT(i,j)=cvGet2D(&_R,i,j).val[0];
	mrpt::math::TPoint3D mT;
	mT.x = cvGet2D(&_T,0,0).val[0];
	mT.y = cvGet2D(&_T,1,0).val[0];
	mT.z = cvGet2D(&_T,2,0).val[0];

	// NOTE: OpenCV seems to return the inverse of what we want, so invert the pose:
	const mrpt::poses::CPose3D     RT_YPR(-mrpt::poses::CPose3D(mROT,mT));
	const mrpt::poses::CPose3DQuat RT_quat(RT_YPR);

	fprintf( f_out,
		"# Relative pose of the right camera wrt to the left camera:\n"
		"[CAMERA_PARAMS_LEFT2RIGHT_POSE]\n"
		"translation_only     = [%e %e %e]\n"
		"rotation_matrix_only = %s\n"
		"pose_yaw_pitch_roll  = %s\n"
		"pose_quaternion      = %s\n\n"
		,
			RT_YPR.x(),RT_YPR.y(),RT_YPR.z(),
			RT_YPR.getRotationMatrix().inMatlabFormat(13).c_str(),
			RT_YPR.asString().c_str(),
			RT_quat.asString().c_str()
		);


	// Convert RT to MRPT classes:
	mrpt::math::CMatrixFixedNumeric<double,3,3> mR1, mR2;
	mrpt::math::CMatrixFixedNumeric<double,3,4> mP1, mP2;
	mrpt::math::CMatrixFixedNumeric<double,4,4> mQ;
	for (int i=0;i<3;i++)
		for (int j=0;j<3;j++)
		{
			mR1(i,j)=R1[i][j];
			mR2(i,j)=R2[i][j];
		}
	for (int i=0;i<3;i++)
		for (int j=0;j<4;j++)
		{
			mP1(i,j)=P1[i][j];
			mP2(i,j)=P2[i][j];
		}
	for (int i=0;i<4;i++)
		for (int j=0;j<4;j++)
			mQ(i,j)=Q[i][j];

	fprintf( f_out,
		"# Stereo rectify matrices (see http://opencv.willowgarage.com/documentation/camera_calibration_and_3d_reconstruction.html ):\n"
		"# R1, R2: The output 3x3 rectification transforms (rotation matrices) for the first and the second cameras, respectively.\n"
		"# P1, P2: The output 3x4 projection matrices in the new (rectified) coordinate systems.\n"
		"[STEREO_RECTIFY_MATRICES]\n"
		"R1 = %s\n"
		"R2 = %s\n"
		"P1 = %s\n"
		"P2 = %s\n\n"
		"Q  = %s\n\n"
		,
		mR1.inMatlabFormat(13).c_str(),
		mR2.inMatlabFormat(13).c_str(),
		mP1.inMatlabFormat(13).c_str(),
		mP2.inMatlabFormat(13).c_str(),
		mQ.inMatlabFormat(13).c_str()
		);



	fprintf( f_out,
		"# Info about calibration parameters:\n"
		"[CALIB_METAINFO]\n"
		"number_good_cheesboards = %i\n"
		"average_reprojection_error = %f // pixels\n"
		"cheesboard_nx = %i\n"
		"cheesboard_ny = %i\n"
		"cheesboard_square_size = %f\n"
		"alpha = %f // Parameter for zoom in/out\n"
		"flag_fix_aspect_ratio = %s\n"
		"flag_zero_tangent_dist = %s\n"
		"flag_same_focal_len = %s\n\n"
		,
		nframes,
		avgErr/(nframes*n),
		nx,ny,
		(double)squareSize,
		alpha,
		flag_fix_aspect_ratio ? "true":"false",
		flag_zero_tangent_dist ? "true":"false",
		flag_same_focal_len ? "true":"false"
		);

	fprintf( f_out,
		"# List of files used in the optimization:\n"
		"[CALIB_FILE_LIST]\n");
	for (unsigned int i=0;i<lst_image_files.size();i++)
		fprintf( f_out, "img_%04u = %s\n", i, lst_image_files[i].c_str() );

	fprintf( f_out, "\n");

	fclose(f_out);

	// DISPLAY RECTIFICATION
	// ========================================================
    cvNamedWindow( "rectified", 1 );
// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
    if( !isVerticalStereo )
        pair = cvCreateMat( imageSize.height, imageSize.width*2,
        CV_8UC3 );
    else
        pair = cvCreateMat( imageSize.height*2, imageSize.width,
        CV_8UC3 );
//Setup for finding stereo corrrespondences
    CvStereoBMState *BMState = cvCreateStereoBMState();
    assert(BMState != 0);
    BMState->preFilterSize=41;
    BMState->preFilterCap=31;
    BMState->SADWindowSize=41;
    BMState->minDisparity=-64;
    BMState->numberOfDisparities=128;
    BMState->textureThreshold=10;
    BMState->uniquenessRatio=15;
    for( i = 0; i < nframes; i++ )
    {
        IplImage* img1=cvLoadImage(imageNames[0][i].c_str(),0);
        IplImage* img2=cvLoadImage(imageNames[1][i].c_str(),0);
        if( img1 && img2 )
        {
            CvMat part;
            cvRemap( img1, img1r, mx1, my1 );
            cvRemap( img2, img2r, mx2, my2 );

            if( !isVerticalStereo || useUncalibrated != 0 )
            {
          // When the stereo camera is oriented vertically,
          // useUncalibrated==0 does not transpose the
          // image, so the epipolar lines in the rectified
          // images are vertical. Stereo correspondence
          // function does not support such a case.
                cvFindStereoCorrespondenceBM( img1r, img2r, disp,
                    BMState);
                cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
                //cvNamedWindow( "disparity" );
                //cvShowImage( "disparity", vdisp );
            }
            if( !isVerticalStereo )
            {
                cvGetCols( pair, &part, 0, imageSize.width );
                cvCvtColor( img1r, &part, CV_GRAY2BGR );
                cvGetCols( pair, &part, imageSize.width,
                    imageSize.width*2 );
                cvCvtColor( img2r, &part, CV_GRAY2BGR );
                for( j = 0; j < imageSize.height; j += 16 )
                    cvLine( pair, cvPoint(0,j),
                    cvPoint(imageSize.width*2,j),
                    CV_RGB(0,255,0));
            }
            else
            {
                cvGetRows( pair, &part, 0, imageSize.height );
                cvCvtColor( img1r, &part, CV_GRAY2BGR );
                cvGetRows( pair, &part, imageSize.height,
                    imageSize.height*2 );
                cvCvtColor( img2r, &part, CV_GRAY2BGR );
                for( j = 0; j < imageSize.width; j += 16 )
                    cvLine( pair, cvPoint(j,0),
                    cvPoint(j,imageSize.height*2),
                    CV_RGB(0,255,0));
            }
            cvShowImage( "rectified", pair );
            if( cvWaitKey() == 27 )
                break;
        }
        cvReleaseImage( &img1 );
        cvReleaseImage( &img2 );
    }
    cvReleaseStereoBMState(&BMState);
    cvReleaseMat( &mx1 );
    cvReleaseMat( &my1 );
    cvReleaseMat( &mx2 );
    cvReleaseMat( &my2 );
    cvReleaseMat( &img1r );
    cvReleaseMat( &img2r );
    cvReleaseMat( &disp );

}

#endif // MRPT_HAS_OPENCV

int main(void)
{
#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x100
	//CvMat leftCamMat, rightCamMat;
	//CvMat leftDisCoe, rightDisCoe;
    //StereoCalib("stereo_calib.txt", 6, 7, 1, &leftCamMat, &leftDisCoe, &rightCamMat, &rightDisCoe );
	//StereoCalib("stereo_calib.txt", 6, 9, 0 );


	string sInFile;
	cout << "This program expects an input text file with the list of images, one image per line alternating left/right/left/right/...\n";
	cout << "File with list of images [ENTER=stereo_calib.txt]: ";
	std::getline(cin,sInFile);
	if (sInFile.empty())
		sInFile="stereo_calib.txt";

	int nx = 6;
	int ny = 9;
	cout << "Now enter the number of rows & columns of cross-points in your checkerboard.\n";
	cout << "Number of horizontal cross points (nx): "; cin >> nx;
	cout << "Number of vertical cross points (ny)  : "; cin >> ny;

    float squareSize = 0.034f;
	cout << "Length of squares (in meters, e.g. 0.034 = 3.4cm) : "; cin >> squareSize;

	double alpha = -1;
	cout <<
		"Enter 'alpha' scaling parameter:\n"
		" -1 => Auto\n"
		" Range [0,1]: \n"
		"   0 => the entire image is valid pixels (zoom in)\n"
		"   1 => don't discard any original pixel (zoom out)\n";
	cout << "Alpha : "; cin >> alpha;

	const bool flag_fix_aspect_ratio  = askBooleanQuestion("Force fixed aspect ratio, that is, fx=fy? [NO/yes]: ", false);
	const bool flag_zero_tangent_dist = askBooleanQuestion("Force zero tangent distortion? [no/YES]: ", true);
	const bool flag_same_focal_len    = askBooleanQuestion("Force both cameras have same focal length? [NO/yes]: ", false);

	string sOutFile;
	cout << "Output file for report of resuls [ENTER=stereo_calib_report.txt]: ";
	std::getline(cin,sOutFile );
	if (sOutFile .empty())
		sOutFile = "stereo_calib_report.txt";

	StereoCalib(sInFile.c_str(),sOutFile.c_str(), nx,ny, 0, squareSize, alpha, flag_fix_aspect_ratio, flag_zero_tangent_dist, flag_same_focal_len );
#else
	printf("OpenCV 1.1.0 or above required.\n");
#endif
    return 0;
}
