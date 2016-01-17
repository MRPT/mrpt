/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/slam.h>
#include <mrpt/stereoslam.h>
#include <mrpt/features.h> 	// For feature detection, etc.
#include <mrpt/gui.h>		// For visualization windows
#include <mrpt/hwdrivers.h>	// For capture of video from videos/cameras

#       include <opencv2/core/core.hpp>
#		include <opencv2/highgui/highgui.hpp>
#		include <opencv2/imgproc/imgproc.hpp>
#		include <opencv2/imgproc/imgproc_c.h>
#		include <opencv2/features2d/features2d.hpp>
#		include <opencv2/video/tracking.hpp>
#		include <opencv2/calib3d/calib3d.hpp>
#		include <opencv2/objdetect/objdetect.hpp>

#define MRPT_EXAMPLES_BASE_DIRECTORY "/home/darkown/work/mrpt-svn/samples/"

//#include "cv.h"
//#include "cxmisc.h"
//#include "highgui.h"

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::vision;
using namespace mrpt::stereoslam;
using namespace mrpt::hwdrivers;
using namespace std;

//*****************************************************
//			Config params
//*****************************************************/
string					INI_FILENAME;
utils::CConfigFile		*iniFile = NULL;

string					RAWLOG_FILE;
unsigned int			RAWLOG_OFFSET;
string					OUT_DIR_STD;
const char				*OUT_DIR;
int						LOG_FREQUENCY;
unsigned int			DECIMATION;

void vOdometry_onthefly()
{
    // LOAD INTRINSIC AND DISTORTION PARAMS OF THE CAMERAS FROM CONFIG FILE
    TCamera leftCamera, rightCamera;
    leftCamera.loadFromConfigFile( "LEFT_CAMERA", *iniFile );
    rightCamera.loadFromConfigFile( "RIGHT_CAMERA", *iniFile );

    const int resX = leftCamera.ncols;
    const int resY = leftCamera.nrows;

    vector_double rcTrans(3);
    vector_double rcRot(9);
	iniFile->read_vector( "RIGHT_CAMERA", "rcTrans", vector_double(), rcTrans, true );
	iniFile->read_vector( "RIGHT_CAMERA", "rcRot", vector_double(), rcRot, true );

    double m1[3][3];
    for(unsigned int i = 0; i < 3; ++i)
        for(unsigned int j = 0; j < 3; ++j)
            m1[i][j] = rcRot[i*3+j];

    // PRE: COMPUTE INITIAL STEREO RECTIFICATION MAPS
    double ipl[3][3], ipr[3][3], dpl[5], dpr[5];
    for( unsigned int i = 0; i < 3; ++i )
        for( unsigned int j = 0; j < 3; ++j )
        {
            ipl[i][j] = leftCamera.intrinsicParams(i,j);
            ipr[i][j] = rightCamera.intrinsicParams(i,j);
        }

    for( unsigned int i = 0; i < 5; ++i )
    {
        dpl[i] = leftCamera.dist[i];
        dpr[i] = rightCamera.dist[i];
    }

    // WITH OLD OPENCV VERSION
    // *****************************************************************
    /** /
    CvMat R = cvMat( 3, 3, CV_64F, &m1 );
    CvMat T = cvMat( 3, 1, CV_64F, &rcTrans );

    CvMat K1 = cvMat(3,3,CV_64F,ipl);
    CvMat K2 = cvMat(3,3,CV_64F,ipr);
    CvMat D1 = cvMat(1,5,CV_64F,dpl);
    CvMat D2 = cvMat(1,5,CV_64F,dpr);

    double _R1[3][3], _R2[3][3], _P1[3][4], _P2[3][4];
    CvMat R1 = cvMat(3,3,CV_64F,_R1);
    CvMat R2 = cvMat(3,3,CV_64F,_R2);
    CvMat P1 = cvMat(3,4,CV_64F,_P1);
    CvMat P2 = cvMat(3,4,CV_64F,_P2);

    CvSize imageSize = {resX,resY};
    cvStereoRectify( &K1, &K2, &D1, &D2, imageSize,
        &R, &T,
        &R1, &R2, &P1, &P2, 0, 0 );

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

    cvInitUndistortRectifyMap(&K1,&D1,&R1,&P1,mx1,my1);
    cvInitUndistortRectifyMap(&K2,&D2,&R2,&P2,mx2,my2);

    CImage im1, im2;
    im1.loadFromFile("imgs/leftImage1024.jpg");
    im2.loadFromFile("imgs/rightImage1024.jpg");

    cvRemap( static_cast<IplImage *>( im1.getAsIplImage() ), img1r, mx1, my1 );
    cvRemap( static_cast<IplImage *>( im2.getAsIplImage() ), img2r, mx2, my2 );

    cvSaveImage( "imgs/leftImage1024_rect.jpg", img1r );
    cvSaveImage( "imgs/rightImage1024_rect.jpg", img2r );

    IplImage stub, *dst_img, stub2, *dst_img2;
    dst_img = cvGetImage(img1r, &stub);
    dst_img2 = cvGetImage(img2r, &stub2);

    CImage im1out( dst_img );
    CImage im2out( dst_img2 );
    /**/
    // *****************************************************************

    // WITH NEW OPENCV VERSION
    // *****************************************************************
    /**/
    cv::Mat R( 3, 3, CV_64F, &m1 );
    cv::Mat T( 3, 1, CV_64F, &rcTrans );

    cv::Mat K1(3,3,CV_64F,ipl);
    cv::Mat K2(3,3,CV_64F,ipr);
    cv::Mat D1(1,5,CV_64F,dpl);
    cv::Mat D2(1,5,CV_64F,dpr);

    double _R1[3][3], _R2[3][3], _P1[3][4], _P2[3][4], _Q[4][4];
    cv::Mat R1(3,3,CV_64F,_R1);
    cv::Mat R2(3,3,CV_64F,_R2);
    cv::Mat P1(3,4,CV_64F,_P1);
    cv::Mat P2(3,4,CV_64F,_P2);
    cv::Mat Q(4,4,CV_64F,_Q);

    cv::Size nSize(resX,resY);
    float alpha = 0.0;                  // alpha value: 0.0 = zoom and crop the image so there's not black areas
    cv::stereoRectify(
        K1, D1,
        K2, D2,
        nSize,
        R, T,
        R1, R2, P1, P2, Q, alpha,
        cv::Size(), 0, 0, 0 );

    cv::Mat mapx1, mapy1, mapx2, mapy2;
    mapx1.create( resY, resX, CV_32FC1 );
    mapy1.create( resY, resX, CV_32FC1 );
    mapx2.create( resY, resX, CV_32FC1 );
    mapy2.create( resY, resX, CV_32FC1 );

    cv::Size sz1, sz2;
    cv::initUndistortRectifyMap( K1, D1, R1, P1, cv::Size(resX,resY), CV_32FC1, mapx1, mapy1 );
    cv::initUndistortRectifyMap( K2, D2, R2, P2, cv::Size(resX,resY), CV_32FC1, mapx2, mapy2 );

    // Capture image from camera
    // Perform visual odometry!!
    CVisualOdometryStereo vOdometer;

    vOdometer.loadOptions( *iniFile );
    vOdometer.stereoParams.dumpToConsole();

    mrpt::system::pause();

    CCameraSensorPtr cam( new CCameraSensor );

    cam->loadConfig( *iniFile, "GRABBER_CONFIG");
    cam->initialize();	// This will raise an exception if neccesary

//    // Extraction
//    CFeatureExtraction fExt;
//    fExt.options.loadFromConfigFile( *iniFile, "EXTRACTION" );
//    fExt.options.dumpToConsole();
//
//    // Matching
//    TMatchingOptions matchOptions;
//    matchOptions.loadFromConfigFile( *iniFile, "MATCH" );
//    matchOptions.dumpToConsole();
//
//    // Projection
//    TStereoSystemParams stereoParams;
//    stereoParams.loadFromConfigFile( *iniFile, "PROJECT" );
//    stereoParams.dumpToConsole();

//    CDisplayWindow w1, w2, w3, w4;  // The grabber and rectified images
//    w1.setPos(0,0);
//    w2.setPos(340,0);
//    w3.setPos(0,300);
//    w4.setPos(340,300);

    CDisplayWindow win("Matches");
    win.setPos(0,550);

    CDisplayWindow3D win3D("Map");
    COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
    {	// Ground plane:
		CGridPlaneXYPtr obj = CGridPlaneXY::Create(-200,200,-200,200,0, 5);
		obj->setColor(0.7,0.7,0.7);
		scene->insert(obj);
		scene->insert( stock_objects::CornerXYZ() );
	}
    win3D.setCameraZoom(10);
    win3D.unlockAccess3DScene();
	win3D.repaint();

    CTicTac tictac;
    unsigned int counter = 0;
    // ------------------------------------------------------------------------
    // MAIN LOOP --------------------------------------------------------------
    // ------------------------------------------------------------------------
    while( !mrpt::system::os::kbhit() )
    {
        if( counter == 0 )
            tictac.Tic();

        CObservationPtr obs;
		try	{ obs= cam->getNextFrame();	}
		catch (CExceptionEOF &)	{ /* End of a rawlog file.*/ break; }

		if (!obs)
		{
			cerr << "*Warning* getNextFrame() returned NULL!\n";
			mrpt::system::sleep(50);
			continue;
		}

		CObservationStereoImagesPtr o = CObservationStereoImagesPtr(obs);
		o->leftCamera = leftCamera;
		o->rightCamera = rightCamera;

//		w1.showImage( o->imageLeft );
//		w2.showImage( o->imageRight );

        cv::Mat outMat1( resY, resX, CV_64F );
        cv::Mat outMat2( resY, resX, CV_64F );

        cv::remap( cv::Mat( static_cast<IplImage *>( o->imageLeft.getAsIplImage() ) ), outMat1, mapx1, mapy1, cv::INTER_CUBIC );
        cv::remap( cv::Mat( static_cast<IplImage *>( o->imageRight.getAsIplImage() ) ), outMat2, mapx2, mapy2, cv::INTER_CUBIC );

        IplImage iplim1 = IplImage(outMat1);
        IplImage iplim2 = IplImage(outMat2);

    //    cvSaveImage( "imgs/leftoutimg1024_rect.jpg", &iplim1 );
    //    cvSaveImage( "imgs/rightoutimg1024_rect.jpg", &iplim2 );

        // Insert them into the observation
        o->imageLeft.loadFromIplImage( &iplim1 );
        o->imageRight.loadFromIplImage( &iplim2 );

//        w3.showImage( o->imageLeft );
//        w4.showImage( o->imageRight );

        if( counter%10 == 0 )
        {
            cout << "FPS: " << 10.0/tictac.Tac() << endl;
            tictac.Tic();
        }
        counter++;

        poses::CPose3DQuatPDFGaussian outEst;
        vOdometer.process_light( o, outEst );
        const TOdometryInfo info = vOdometer.getInfo();

        win.showImagesAndMatchedPoints( info.m_obs->imageLeft, info.m_obs->imageRight, info.m_mfList );
        //cout << "OUT: " << outEst.mean << endl;
    /**/
    // *****************************************************************

//
//    cout << "Extracting features with type: " << fExt.options.featsType;
//    cout << " and patch size: " << fExt.options.patchSize << endl;
//
//    CFeatureList leftList, rightList;
//    fExt.detectFeatures( o->imageLeft, leftList );
//    fExt.detectFeatures( o->imageRight, rightList );
//
//    leftList.saveToTextFile( "imgs/leftlist.txt" );
//    rightList.saveToTextFile( "imgs/rightlist.txt" );
//
//    CMatchedFeatureList outMatchedList;
//    unsigned int nMatches = mrpt::vision::matchFeatures( leftList, rightList, outMatchedList, matchOptions );
//    cout << "Left features: " << leftList.size() << endl;
//    cout << "Right features: " << rightList.size() << endl;
//    cout << "Matches: " << nMatches << endl;
//
//    // Show matches
//    win.showImagesAndMatchedPoints( o->imageLeft, o->imageRight, outMatchedList );
//
//    mrpt::maps::CLandmarksMap landmarks;
//    mrpt::vision::projectMatchedFeatures( outMatchedList, stereoParams, landmarks);
//    landmarks.changeCoordinatesReference( CPose3D( 0, 0, 0, DEG2RAD(-90), 0, DEG2RAD(-90) ) );
//
//    COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
//    scene->clear();
//    mrpt::opengl::CSetOfObjectsPtr obj = CSetOfObjects::Create();
//    landmarks.getAs3DObject( obj );
////    if( counter == 0 )
////    {
////        obj->setName( "elipses" );
//        scene->insert( obj );
////    }
////    else
////    {
////        mrpt::opengl::CSetOfObjectsPtr my_obj = static_cast<CSetOfObjectsPtr>(scene->getByName( "elipses" ));
////        my_obj->clear();
////        my_obj->insert( obj );
////        scene->insert(obj);
////    }
////
//    win3D.setCameraZoom(10);
//    win3D.unlockAccess3DScene();
//	win3D.repaint();
//
//    mrpt::system::pause();
// HASTA AQUIII
////    CObservationStereoImages obs;
////    obs.imageLeft.copyFastFrom( im1out );
////    obs.imageRight.copyFastFrom( im2out );
//
//    CVisualOdometryStereo vOdometer;
//    vOdometer.loadOptions( *iniFile );

    // if IT_1
    // COMPUTE FAST FEATURES (ADAPTATIVE FAST THRESHOLD)
    // MATCH WITH SAD
    // PROJECT TO 3D
    // STORE

    // if IT_i
    // TRACK FEATURES
    // CHECK MATCHES: EPIPOLAR, OUT_OF_BOUNDS
    // PROJECT TO 3D
    // COMPUTE HORN + COVARIANCE (new)

    // ENOUGH FEATURES? -> FIND MORE FEATURES
    // back to [1]
    }
    return;
} // end vOdometry_onthefly()


// ------------------------------------------------------
//					Visual Odometry
// ------------------------------------------------------
void vOdometry_lightweight()
{
	// My Local Variables
	CVisualOdometryStereo				vOdometer;
	unsigned int						step = 0;
	CTicTac								tictac;

	std::vector<CPose3DQuat>			path1;
	std::vector<CPose3DQuatPDFGaussian>	path2;

	size_t								rawlogEntry = 0;
	CFileGZInputStream					rawlogFile( RAWLOG_FILE );

	// Initial pose of the path
	path1.push_back( CPose3DQuat() );
	path2.push_back( CPose3DQuatPDFGaussian() );

	// ----------------------------------------------------------
	//						vOdometry
	// ----------------------------------------------------------
	CActionCollectionPtr	action;
	CSensoryFramePtr		observations;
	CObservationPtr			observation;

	// Load options (stereo + matching + odometry)
	vOdometer.loadOptions( INI_FILENAME );

	// Delete previous files and prepare output dir
	deleteFiles( format("%s/*.txt", OUT_DIR) );

	FILE *f_cov = os::fopen( format( "%s/cov.txt", OUT_DIR ), "wt");
	ASSERT_( f_cov != NULL );

	// Iteration counter
	int	counter = 0;

	FILE *f_log = os::fopen( format( "%s/q.txt", OUT_DIR ), "wt");
	FILE *f_log2 = os::fopen( format( "%s/path.txt", OUT_DIR ), "wt");

	unsigned int imDecimation = 5;

	// Main Loop
	tictac.Tic();
	for (;;)
	{
		if (os::kbhit())
		{
			char c = os::getch();
			if (c==27)
				break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::getActionObservationPairOrObservation( rawlogFile, action, observations, observation, rawlogEntry) )
			break; // file EOF


		if ( rawlogEntry >= RAWLOG_OFFSET && 0 == ( step % DECIMATION ) )
		{
			// Execute:
			// ----------------------------------------

			// STEREO IMAGES OBSERVATION
			CObservationStereoImagesPtr sImgs;
			if( observation )
				sImgs = CObservationStereoImagesPtr( observation );
			else
				sImgs = observations->getObservationByClass<CObservationStereoImages>();

			poses::CPose3DQuatPDFGaussian outEst;
			if( sImgs )
			{
				if( counter == 0 )
				{
					// Set initial parameters
					vOdometer.stereoParams.baseline = sImgs->rightCameraPose.x();
					vOdometer.stereoParams.K = sImgs->leftCamera.intrinsicParams;
				}

				cout << "Rawlog Entry: " << rawlogEntry << " Iteration: " << counter++ << endl;

				if( step % imDecimation )
				{
					vOdometer.process_light( sImgs, outEst );
					TOdometryInfo info = vOdometer.getInfo();

					// Save to file both the quaternion and covariance matrix
					os::fprintf( f_log,"%f %f %f %f %f %f %f\n",
						outEst.mean[0], outEst.mean[1], outEst.mean[2], outEst.mean[3], outEst.mean[4], outEst.mean[5], outEst.mean[6] );
					info.m_Prev_cloud.landmarks.saveToTextFile( format( "%s/clouds%04d.txt", OUT_DIR, step ) );

					path1.push_back( path1.back() + outEst.mean );
					os::fprintf( f_log2,"%f %f %f %f %f %f %f\n",
						path1.back()[0], path1.back()[1], path1.back()[2], path1.back()[3], path1.back()[4], path1.back()[5], path1.back()[6] );

				path2.push_back( outEst );
				}
				else
					cout << "Skipped step" << endl;

			} // end if sImgs != NULL

		} // end if 'rawlogEntry >= rawlog_offset'

		step++;

		// Free memory:
		action.clear_unique();
		observations.clear_unique();

	}; // end  while !end
	cout << "*************** Tiempo: " << 1000.0f*tictac.Tac() << "************************" << endl;

	os::fclose( f_cov );
	os::fclose( f_log );
	os::fclose( f_log2 );

	// SAVE THE RESULTS
	/**/
	FILE *fPath1 = os::fopen( format("./%s/EstimatedPath.txt", OUT_DIR).c_str(), "wt");
	if( fPath1 != NULL )
	{
		std::vector<CPose3DQuat>::iterator	itPath;
		for(itPath = path1.begin(); itPath != path1.end(); ++itPath )
			os::fprintf( fPath1,"%f %f %f %f %f %f %f\n",
			itPath->x(), itPath->y(), itPath->z(),
			itPath->quat().r(), itPath->quat().x(), itPath->quat().y(), itPath->quat().z() );

		os::fclose( fPath1 );
	}
	else
		std::cout << "WARNING: The estimated path could not be saved" << std::endl;

	FILE *fPath2 = os::fopen( format("./%s/EstimatedPathPDF.txt", OUT_DIR).c_str(), "wt");
	if( fPath2 != NULL )
	{
		std::vector<CPose3DQuatPDFGaussian>::iterator	itPath;
		for(itPath = path2.begin(); itPath != path2.end(); ++itPath )
		{
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
				itPath->mean.x(), itPath->mean.y(), itPath->mean.z(),
				itPath->mean.quat().r(), itPath->mean.quat().x(), itPath->mean.quat().y(), itPath->mean.quat().z() );

			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(0,0), itPath->cov.get_unsafe(0,1), itPath->cov.get_unsafe(0,2), itPath->cov.get_unsafe(0,3), itPath->cov.get_unsafe(0,4), itPath->cov.get_unsafe(0,5), itPath->cov.get_unsafe(0,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(1,0), itPath->cov.get_unsafe(1,1), itPath->cov.get_unsafe(1,2), itPath->cov.get_unsafe(1,3), itPath->cov.get_unsafe(1,4), itPath->cov.get_unsafe(1,5), itPath->cov.get_unsafe(1,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(2,0), itPath->cov.get_unsafe(2,1), itPath->cov.get_unsafe(2,2), itPath->cov.get_unsafe(2,3), itPath->cov.get_unsafe(2,4), itPath->cov.get_unsafe(2,5), itPath->cov.get_unsafe(2,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(3,0), itPath->cov.get_unsafe(3,1), itPath->cov.get_unsafe(3,2), itPath->cov.get_unsafe(3,3), itPath->cov.get_unsafe(3,4), itPath->cov.get_unsafe(3,5), itPath->cov.get_unsafe(3,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(4,0), itPath->cov.get_unsafe(4,1), itPath->cov.get_unsafe(4,2), itPath->cov.get_unsafe(4,3), itPath->cov.get_unsafe(4,4), itPath->cov.get_unsafe(4,5), itPath->cov.get_unsafe(4,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(5,0), itPath->cov.get_unsafe(5,1), itPath->cov.get_unsafe(5,2), itPath->cov.get_unsafe(5,3), itPath->cov.get_unsafe(5,4), itPath->cov.get_unsafe(5,5), itPath->cov.get_unsafe(5,6));
			os::fprintf(fPath2,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", itPath->cov.get_unsafe(6,0), itPath->cov.get_unsafe(6,1), itPath->cov.get_unsafe(6,2), itPath->cov.get_unsafe(6,3), itPath->cov.get_unsafe(6,4), itPath->cov.get_unsafe(6,5), itPath->cov.get_unsafe(6,6));
		}
		os::fclose( fPath2 );
	}
	else
		std::cout << "WARNING: The estimated pdf path could not be saved" << std::endl;

	std::cout << "Saved results!" << std::endl;
	/**/

	mrpt::system::pause();

}

// ------------------------------------------------------
//					Visual Odometry
// ------------------------------------------------------
//void vOdometry()
//{
//	// My Local Variables
//	CVisualOdometryStereo				vOdometer;
//	unsigned int						step = 0;
//
//	std::vector<CPose3DQuat>			path;
//
//	// Other variables
//	CTicTac								tictac;
//
//	size_t								rawlogEntry = 0;
//	CFileGZInputStream					rawlogFile( RAWLOG_FILE );
//
//	CDisplayWindow3D					win( "Visual Odometry", 640, 480 );
//	win.setPos( 700, 20 );
//
//	COpenGLScenePtr						&theScene = win.get3DSceneAndLock();
//
//	// OPENGL VARIABLES
//	{
//		opengl::CPointCloudPtr	obj = opengl::CPointCloud::Create();
//		obj->setName("path");
//		theScene->insert( obj );
//	}
//	{
//		opengl::CSetOfLinesPtr	obj = opengl::CSetOfLines::Create();
//		obj->setName("path_lines");
//		theScene->insert( obj );
//	}
//
//	{
//		opengl::CGridPlaneXYPtr	obj = opengl::CGridPlaneXY::Create(-0.25,0.25,-0.5,0.5,0,0.05);
//		obj->setColor(0.4,0.4,0.4);
//		theScene->insert( obj );
//	}
//	{
//		opengl::CEllipsoidPtr obj = opengl::CEllipsoid::Create();
//		obj->setName("ellipse");
//		theScene->insert( obj );
//	}
//	{
//		opengl::CAxisPtr obj = opengl::CAxis::Create();
//		obj->setFrequency(5);
//		obj->enableTickMarks();
//		obj->setAxisLimits(-10,-10,-10, 10,10,10);
//		theScene->insert( obj );
//	}
//
//	win.setCameraElevationDeg( 25.0f );
//	win.setCameraAzimuthDeg( 25.0f );
//	win.setCameraZoom( 1.0f );
//
//	win.unlockAccess3DScene();
//
//	// Initial pose of the path
//	path.push_back( CPose3DQuat() );
//
//	// ----------------------------------------------------------
//	//						vOdometry
//	// ----------------------------------------------------------
//	CActionCollectionPtr	action;
//	CSensoryFramePtr		observations;
//
//	// Load options
//	vOdometer.stereoParams.loadFromConfigFile(*iniFile,"StereoParams");
//	vOdometer.stereoParams.dumpToConsole();
//	vOdometer.matchingOptions.loadFromConfigFile(*iniFile,"MatchingOptions"	);
//	vOdometer.matchingOptions.dumpToConsole();
//	vOdometer.odometryOptions.loadFromConfigFile(*iniFile,"OdometryOptions");
//	vOdometer.odometryOptions.dumpToConsole();
//
//	// Delete previous files
//	deleteFiles( format("%s/*.txt", OUT_DIR) );
//
//	FILE *f_cov = os::fopen( format( "%s/cov.txt", OUT_DIR ), "wt");
//	ASSERT_( f_cov != NULL );
//
//	// Main Loop
//	for (;;)
//	{
//		if (os::kbhit())
//		{
//			char c = os::getch();
//			if (c==27)
//				break;
//		}
//
//		// Load action/observation pair from the rawlog:
//		// --------------------------------------------------
//		if (! CRawlog::readActionObservationPair( rawlogFile, action, observations, rawlogEntry) )
//			break; // file EOF
//
//		if ( rawlogEntry >= rawlog_offset )
//		{
//			// Execute:
//			// ----------------------------------------
//
//			// STEREO IMAGES OBSERVATION
//			CObservationStereoImagesPtr sImgs = observations->getObservationByClass<CObservationStereoImages>();
//			poses::CPose3DQuatPDFGaussian outEst;
//			if( sImgs )
//			{
//				// Set initial parameters
//				vOdometer.stereoParams.baseline = sImgs->rightCameraPose.x();
//				sImgs->leftCamera.getIntrinsicParamsMatrix( vOdometer.stereoParams.K );
//
//				// Perform an iteration
//				if(rawlogEntry > 200)
				//	mrpt::system::pause();
//
//				vOdometer.process( sImgs, outEst );
//
//				CPose3D	thispose = path.at( path.size() - 1 ) + outEst.mean;
//				path.push_back( thispose );
//
//				os::fprintf(f_cov, "%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
//							outEst.cov.get_unsafe(0,0), outEst.cov.get_unsafe(0,1), outEst.cov.get_unsafe(0,2),
//							outEst.cov.get_unsafe(1,0), outEst.cov.get_unsafe(1,1), outEst.cov.get_unsafe(1,2),
//							outEst.cov.get_unsafe(2,0), outEst.cov.get_unsafe(2,1), outEst.cov.get_unsafe(2,2));
//
//				COpenGLScenePtr &theScene = win.get3DSceneAndLock();
//
//				{	// COVARIANCE MATRIX
//					CMatrixDouble33 loc_cov = CMatrixDouble33( outEst.cov );
//					//outEst.cov.extractMatrix(0,0,loc_cov);
//					opengl::CEllipsoidPtr obj1 = static_cast<opengl::CEllipsoidPtr>(theScene->getByName("ellipse"));
//					obj1->setCovMatrix(loc_cov);
//					obj1->setPose(thispose);
//					obj1->enableDrawSolid3D(false);
//				}
//
//				{	// POINT CLOUD
//					std::vector<CPose3D>::reverse_iterator	rit = path.rbegin();
//					size_t									tam = path.size();
//
//					opengl::CPointCloudPtr obj1 = static_cast<opengl::CPointCloudPtr>(theScene->getByName("path"));
//
//					obj1->getArrayX().resize( tam );
//					obj1->getArrayY().resize( tam );
//					obj1->getArrayZ().resize( tam );
//
//					obj1->getArrayX()[tam-1]	= rit->x();
//					obj1->getArrayY()[tam-1]	= rit->y();
//					obj1->getArrayZ()[tam-1]	= rit->z();
//
//					obj1->setColor(1,0,0);
//					obj1->setPointSize(3);
//				}
//
//				{	// SET OF LINES
//					size_t									tam = path.size();
//
//					if( tam > 1 )
//					{
//						opengl::CSetOfLinesPtr obj1 = static_cast<opengl::CSetOfLinesPtr>(theScene->getByName("path_lines"));
//
//						std::vector<CPose3D>::reverse_iterator	rit = path.rbegin();
//
//						float x1,y1,z1;
//						x1 = rit->x();
//						y1 = rit->y();
//						z1 = rit->z();
//
//						rit++;
//
//						obj1->appendLine( rit->x(), rit->y(), rit->z(), x1, y1, z1 );
//
//						obj1->setColor(0,0,1);
//					} // end if
//				}
//
//				// UNLOCK SCENE
//				win.unlockAccess3DScene();
//				win.forceRepaint();
//
//				//if( rawlogEntry > 180 )
//				//	mrpt::system::pause();
//
//			} // end if sImgs != NULL
//
//			// DEBUG: PLOT IMAGES AND FEATURES!
//			/** /
//			// DIBUJAR LAS FEATURES
//			if( PLOT_IMG )
//			{
//				char str[10];
//				tmpImg1 = sImgs->imageLeft;
//				if( PLOT_INFO )
//				{
//					for( itSIFT = SIFTmatchL.begin(); itSIFT != SIFTmatchL.end(); itSIFT++)
//					{
//						os::sprintf( str, 10, "%d", (int)itSIFT->ID );
//						tmpImg1.textOut( (int)itSIFT->x + 3, (int)itSIFT->y, str, 0xFF00FF );
//						tmpImg1.cross( (int)itSIFT->x, (int)itSIFT->y, 0xFF0000, '+');
//					}
//
//				} // end if
//
//				wind1.setPos( 0, 400 );
//				wind1.showImage( tmpImg1 );
//
//				tmpImg2 = sImgs->imageRight;
//				if( PLOT_INFO )
//				{
//					//for( itKLT = KLTList2.begin(); itKLT != KLTList2.end(); itKLT++)
//					//	tmpImg2.cross( (int)itKLT->x, (int)itKLT->y, 0x0000FF, '+');
//					for( itSIFT = SIFTmatchR.begin(); itSIFT != SIFTmatchR.end(); itSIFT++)
//					{
//						os::sprintf( str, 10, "%d", (int)itSIFT->ID );
//						tmpImg2.textOut( (int)itSIFT->x + 3, (int)itSIFT->y, str, 0xFF00FF );
//						tmpImg2.cross( (int)itSIFT->x, (int)itSIFT->y, 0xFF0000, '+');
//					}
//
//				} // end if
//
//				wind2.setPos( 650, 400 );
//				wind2.showImage( tmpImg2 );
//			} // end if PLOT_IMGS
//			/ * */
//		} // end if 'rawlogEntry >= rawlog_offset'
//
//		step++;
//
//		// Free memory:
//		action.clear_unique();
//		observations.clear_unique();
//
//	}; // end  while !end
//
//	os::fclose( f_cov );
//
//	// SAVE THE RESULTS
//	FILE *fPath = os::fopen( format("./%s/EstimatedPath.txt", OUT_DIR).c_str(), "wt");
//
//	//if( fPath != NULL )
//	//{
//	//	std::vector<CPose3DQuat>::iterator	itPath;
//	//	for(itPath = path.begin(); itPath != path.end(); itPath++ )
//	//		os::fprintf( fPath,"%f %f %f %f %f %f %f\n",
//	//			itPath->x(), itPath->y(), itPath->z(),
//	//			itPath->quat().r(), itPath->quat().x(), itPath->quat().y(), itPath->quat().z() );
//
//	//	os::fclose( fPath );
//	//}
//
//	mrpt::system::pause();
//} // end vOdometry

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
int main(int argc, char **argv)
{
	try
	{

		// Process arguments:
		if (argc<2)
		{
			printf("Use: MapBuilding_from_Rawlog <CONFIG_FILE.INI>\n\n");
			return -1;
		}

		INI_FILENAME = std::string( argv[1] );
		ASSERT_FILE_EXISTS_(INI_FILENAME)

		iniFile = new utils::CConfigFile( INI_FILENAME );

		// ------------------------------------------
		//			Load config from file:
		// ------------------------------------------
//		RAWLOG_FILE			= iniFile->read_string("OdometryApplication", "rawlogFile", "");
//		RAWLOG_OFFSET		= iniFile->read_int("OdometryApplication", "rawlogOffset", 0);
//		OUT_DIR_STD			= iniFile->read_string("OdometryApplication", "logOutputDir", "LOG_VODOMETRY");
//		LOG_FREQUENCY		= iniFile->read_int("OdometryApplication", "logFrequency", 10);
//		DECIMATION			= iniFile->read_int("OdometryApplication", "decimation", 1);

		// Set relative path for externally-stored images in rawlogs:
//		string	rawlog_images_path = extractFileDirectory( RAWLOG_FILE );
//		rawlog_images_path+=extractFileName(RAWLOG_FILE);
//		rawlog_images_path+="_Images";
//		CImage::IMAGES_PATH_BASE = rawlog_images_path;		// Set it.

//		// For ease!
//		OUT_DIR = OUT_DIR_STD.c_str();
//		createDirectory(OUT_DIR);

//		// Checks:
//		ASSERT_(RAWLOG_FILE.size()>0);
//		ASSERT_(fileExists(RAWLOG_FILE));

//		// Print params:
//		printf(" Running with the following parameters:\n");
//		printf(" RAWLOG file:'%s'\n", RAWLOG_FILE.c_str());
//		printf(" Output directory:\t\t\t'%s'\n",OUT_DIR);
//		printf(" Log record freq:\t\t\t%u\n",LOG_FREQUENCY);

		// Call to the vOdometry process
		// vOdometry();
		vOdometry_lightweight();
        // vOdometry_onthefly();

		delete iniFile;

		mrpt::system::pause();
		return 0;
	} catch (std::exception &e)
	{
		printf("%s",e.what());
		printf("Program finished for an exception!!\n");;
		mrpt::system::pause();
		return -1;
	}
	catch (...)
	{
		printf("Not handled exception!!");
		mrpt::system::pause();
		return -1;
    }
}

