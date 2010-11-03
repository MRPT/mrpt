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
#include <mrpt/slam.h>
#include <mrpt/stereoslam.h>
#include <mrpt/vision.h> 	// For feature detection, etc.
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

void detectFASTinSequence()
{
    // General
    bool usingRawlog = false;

    // Display
    CDisplayWindow win1("Initial set"), win2;
    win1.setPos(0,0);
    win2.setPos(700,0);

    // Extraction
    CFeatureExtraction fExt;
    fExt.options.patchSize                      = 0;
    fExt.options.featsType                      = mrpt::vision::featFAST;
    fExt.options.FASTOptions.threshold          = 10; // 20
    fExt.options.FASTOptions.nonmax_suppression = true; // true
    fExt.options.FASTOptions.min_distance       = 10; // 5
    fExt.options.FASTOptions.use_KLT_response   = true; // false

    CFeatureList list1, list2;
    vector<bool> featureFound;

    // Grab a still rawlog
    // Get the stereo observation
    string img_dir = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/sequence");		// Left image
    if( usingRawlog )
    {
        CFileGZInputStream      rawlogFile("rawlogFile.rawlog");
        CActionCollectionPtr    action;
        CSensoryFramePtr        observations;
        CObservationPtr         observation;
        size_t                  rawlogEntry = 0;
        bool                    end         = false;
        unsigned int            counter     = 0;

        // Read from the rawlog:
        while ( CRawlog::getActionObservationPairOrObservation(
               rawlogFile,      // Input file
               action,          // Possible out var: action of a pair action/obs
               observations,    // Possible out var: obs's of a pair action/obs
               observation,     // Possible out var: a single obs.
               rawlogEntry      // Just an I/O counter
               ) )
        {

            // Process action & observations
            if (observation)
            {
                CObservationStereoImagesPtr stImages = CObservationStereoImagesPtr(observation);

                // Read a single observation from the rawlog (Format #2 rawlog file)
                if( counter == 0 )
                {
                    // Detect FAST features from left image (first frame) and show in a window (stop the processing)
                    fExt.detectFeatures( stImages->imageLeft, list1 );
                    featureFound.resize( list1.size(), false );
                    win1.showImageAndPoints( stImages->imageLeft, list1 );
                    mrpt::system::pause();
                }
                else
                {
                    fExt.detectFeatures( stImages->imageLeft, list2 );

                    // Check the goodness of the finding process
                    CFeatureList::iterator itFeat1, itFeat2;
                    unsigned int k = 0;
                    unsigned int gpoints = 0;
                    for( itFeat1 = list1.begin(); itFeat1 != list1.end(); ++itFeat1, ++k )
                    {
                        // look for its matching in the new frame. As the camera is still, only look for the closest one
                        unsigned int m = 0;
                        for( itFeat2 = list2.begin(); itFeat2 != list2.end(); ++itFeat2, ++m )
                        {
                            if( square((*itFeat1)->x-(*itFeat2)->x) + square((*itFeat1)->y-(*itFeat2)->y) < 16 /*px*/)
                            {
                                // Check the goodness of the feature (if there is still in the same place)
                                featureFound[k] = true;
                                gpoints++;
                                break;
                            } // end if
                        } // end for
                    } // end for
                    // Count the number of good points
                    stImages->imageLeft.textOut( 10, 20, format( "Good features: %d", gpoints), TColor::red );
                    for( unsigned int k = 0; k < list2.size(); ++k )
                    {
                        if( featureFound[k] )
                            stImages->imageLeft.cross( (int)list2[k]->x, (int)list2[k]->y, TColor::green, '+' );
                        else
                            stImages->imageLeft.cross( (int)list2[k]->x, (int)list2[k]->y, TColor::red, '+' );
                    }
                    win2.setWindowTitle( format( "Frame %d", k ) );
                    win2.showImage( stImages->imageLeft );
                } // end else
                counter++;
            }
            else
            {
                // action, observations should contain a pair of valid data (Format #1 rawlog file)
            }
        } // end-while
    }
    else
    {
        // using images
        cout << "Using images..." << endl;
        const unsigned int N = 13;
        CFeatureList list1, list2;
        unsigned int lsize;
        CImage image, initialImage;
        for( unsigned int k = 1; k <= N; k++)
        {
            if( !image.loadFromFile( format( "/home/darkown/work/mrpt-svn/samples/feature_extraction/imgs/sequence/img%d.jpg", k) ) )
            {
                cout << " could not be loaded..." << endl;
                return;
            }
            cout << endl;

            if( k == 1 )
            {
                // Detect FAST features from left image (first frame) and show in a window (stop the processing)
                cout << "Detecting features: ";
                fExt.detectFeatures( image, list1 );
                initialImage = image;
                lsize = list1.size();
                cout << lsize << " detected." << endl;
                win1.showImageAndPoints( image, list1 );
                featureFound.resize( lsize, false );
                mrpt::system::pause();
            } // end-if-k
            else
            {
                fExt.detectFeatures( image, list2 );
                featureFound.assign( featureFound.size(), false );

                // Check the goodness of the finding process
                CFeatureList::iterator itFeat1, itFeat2;
                unsigned int k = 0;
                unsigned int gpoints = 0;
                for( itFeat1 = list1.begin(); itFeat1 != list1.end(); ++itFeat1, ++k )
                {
                    cout << (*itFeat1)->response << endl;
                    // look for its matching in the new frame. As the camera is still, only look for the closest one
                    unsigned int m = 0;
                    for( itFeat2 = list2.begin(); itFeat2 != list2.end(); ++itFeat2, ++m )
                    {
                        if( square((*itFeat1)->x-(*itFeat2)->x) + square((*itFeat1)->y-(*itFeat2)->y) < 16 /*px*/)
                        {
                            // Check the goodness of the feature (if there is still in the same place)
                            featureFound[k] = true;
                            gpoints++;
                            break;
                        } // end if
                    } // end for
                } // end for
                // Count the number of good points
                win2.showImageAndPoints( image, list2 );
                CImage copyImage;
                copyImage = initialImage;
                copyImage.textOut( 10, 450, format( "GF: %d/%d [%.2f%%]", gpoints, lsize, (gpoints*100.0f)/double(lsize) ), TColor::red );
                for( unsigned int k = 0; k < list1.size(); ++k )
                {
                    if( featureFound[k] )
                    {
                        copyImage.cross( list1[k]->x, list1[k]->y, TColor::green, '+' );
                        copyImage.drawCircle( list1[k]->x, list1[k]->y, 4, TColor::green );
                    }
                    else
                    {
                        copyImage.cross( list1[k]->x, list1[k]->y, TColor::red, '+' );
                        copyImage.drawCircle( list1[k]->x, list1[k]->y, 4, TColor::red );
                    }
                }
                win1.showImage( copyImage );
                mrpt::system::pause();
            } // end-else
        } // end-for
    } // end else
} // end-detectFASTinSequence

void orientation_test()
{
    // Capture a pair of images (640x480) ??
    // Rectify them or capture them already rectified
    CImage imageLeft, imageRight;

	string imgL = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imL_p01.jpg");		// Left image
	string imgR = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imR_p01.jpg");		// Right image

    imageLeft.loadFromFile( imgL );
    imageRight.loadFromFile( imgR );

    imageLeft.grayscaleInPlace();
    imageRight.grayscaleInPlace();

    // LOAD INTRINSIC AND DISTORTION PARAMS OF THE CAMERAS FROM CONFIG FILE
    TCamera leftCamera, rightCamera;
    leftCamera.loadFromConfigFile( "LEFT_CAMERA", *iniFile );
    rightCamera.loadFromConfigFile( "RIGHT_CAMERA", *iniFile );

    vector_double rcTrans(3);
    iniFile->read_vector( "RIGHT_CAMERA", "rcTrans", vector_double(), rcTrans, true );

    // ***********************************************
    // RECTIFICATION
    // ***********************************************
    /** /
    imageLeft.loadFromFile("imgs/640x480_left.jpg");
    imageRight.loadFromFile("imgs/640x480_right.jpg");

    unsigned int resX = imageLeft.getWidth();
    unsigned int resY = imageLeft.getHeight();

//    cout << "Left camera: " << leftCamera.dumpAsText() << endl;
//    cout << "Right camera: " << rightCamera.dumpAsText() << endl;

    // Precompute rectification maps
    vector_double rcRot(9);
	iniFile->read_vector( "RIGHT_CAMERA", "rcRot", vector_double(), rcRot, true );

	CMatrixDouble44 auxMatrix;
	for( unsigned int ii = 0; ii < auxMatrix.getColCount(); ++ii )
        for( unsigned int jj = 0; jj < auxMatrix.getRowCount(); ++jj )
            auxMatrix.set_unsafe(jj,ii,rcRot[ii*3+jj]);

	auxMatrix.set_unsafe(0,3,rcTrans[0]);
	auxMatrix.set_unsafe(1,3,rcTrans[1]);
	auxMatrix.set_unsafe(2,3,rcTrans[2]);
	auxMatrix.set_unsafe(3,0,0);
	auxMatrix.set_unsafe(3,1,0);
	auxMatrix.set_unsafe(3,2,0);
	auxMatrix.set_unsafe(3,3,1);

    CPose3D rightCameraPose( auxMatrix );

    cv::Mat mapx1, mapy1, mapx2, mapy2;
    mapx1.create( resY, resX, CV_32FC1 );
    mapy1.create( resY, resX, CV_32FC1 );
    mapx2.create( resY, resX, CV_32FC1 );
    mapy2.create( resY, resX, CV_32FC1 );

    // Precompute the maps
    vision::computeStereoRectificationMaps(
        leftCamera, rightCamera,
        rightCameraPose,
        &mapx1, &mapy1, &mapx2, &mapy2 );

    CDisplayWindow winL_b("Left Before");
    CDisplayWindow winR_b("Right Before");

    winL_b.showImage( imageLeft );
    winR_b.showImage( imageRight );

    // Apply rectification
//    CTicTac tictac;
//    tictac.Tic();
    imageLeft.rectifyImageInPlace( &mapx1, &mapy1 );
    imageRight.rectifyImageInPlace( &mapx2, &mapy2 );

    imageLeft.saveToFile("imgs/640x480_left_rect.jpg");
    imageRight.saveToFile("imgs/640x480_right_rect.jpg");
//    double tend = tictac.Tac();
//    cout << "Time: " << tend*1000.0f << " ms" << endl;

    CDisplayWindow winL_a("Left After");
    CDisplayWindow winR_a("Right After");

    winL_a.showImage( imageLeft );
    winR_a.showImage( imageRight );

    mrpt::system::pause();

    return;
/**/
////    // Projection
////    TStereoSystemParams stereoParams;
////    stereoParams.loadFromConfigFile( *iniFile, "PROJECT" );
////    stereoParams.dumpToConsole();

    // Find (FAST) features
    CFeatureExtraction fExt;
    fExt.options.loadFromConfigFile( *iniFile, "EXTRACTION" );
    fExt.options.dumpToConsole();

    CFeatureList leftFeats,rightFeats;
    fExt.detectFeatures( imageLeft, leftFeats );
    fExt.detectFeatures( imageRight, rightFeats );

    cout << "NFeatures: " << leftFeats.size() << endl;
    cout << "NFeatures: " << rightFeats.size() << endl;

    CDisplayWindow w1("Left features");
    CDisplayWindow w2("Right features");

    w1.showImageAndPoints( imageLeft, leftFeats, TColor::red );
    w2.showImageAndPoints( imageRight, rightFeats, TColor::red );

    CMatchedFeatureList matchedFeats;
    TMatchingOptions options;
    options.loadFromConfigFile( *iniFile, "MatchingOptions" );
    options.dumpToConsole();

//    options.epipolar_TH = 25.0;
//    options.matching_method = TMatchingOptions::mmCorrelation;

    const unsigned int nMatches = vision::matchFeatures( leftFeats, rightFeats, matchedFeats, options );
    cout << "Matches: " << nMatches << endl;

    CDisplayWindow matchWind("Matches");
    matchWind.showImagesAndMatchedPoints( imageLeft, imageRight, matchedFeats, TColor::red );

    std::vector<double>          leftOris,rightOris;        // The orientations
    //unsigned int    patchSize   = fExt.options.patchSize;   // The patch size
    unsigned int    patchSize   = 23;   // The patch size
    unsigned int    counter     = 0;
    unsigned int    goodmatches = 0;
    vector<double>  this_ori;
    CMatchedFeatureList::iterator itMatches;
    FILE *f = mrpt::system::os::fopen( "imgs/descriptors.txt", "wt" );
    CImage aux_leftImg( imageLeft );
    CImage aux_rightImg( imageRight );
    double sigma = 7.5;
    CTicTac ntictac;
    double ti = 0.0;
    char buf[10];
    for( itMatches = matchedFeats.begin(); itMatches != matchedFeats.end(); ++itMatches, ++counter )
    {
        double u1, v1, u2, v2;
        u1 = itMatches->first->x;
        v1 = itMatches->first->y;
        u2 = itMatches->second->x;
        v2 = itMatches->second->y;

        // Left
//        ntictac.Tic();
        vision::computeMainOrientations( imageLeft, (unsigned int)u1, (unsigned int)v1, patchSize, leftOris, sigma );
//        ti += ntictac.Tac();
        for( unsigned int k = 0; k < leftOris.size(); ++k )
        {
            cout << "ORI:" << leftOris[k] << endl;
            aux_leftImg.line( (int)u1, (int)v1, (int)(u1+10*cos(leftOris[k])), (int)(v1+10*sin(leftOris[k])), TColor::red );
            aux_leftImg.drawCircle( (int)u1, (int)v1, 2, TColor::red );
            mrpt::system::os::sprintf(buf, 10, "%d", (int)itMatches->first->ID );
            aux_leftImg.textOut( (int)u1, (int)v1, buf ,TColor::blue );
        }

        vision::computeMainOrientations( imageRight, (unsigned int)u2, (unsigned int)v2, patchSize, rightOris, sigma );
        for( unsigned int k = 0; k < rightOris.size(); ++k )
        {
            cout << "ORI:" << rightOris[k] << endl;
            aux_rightImg.line( (int)u2, (int)v2, (int)(u2+10*cos(rightOris[k])), (int)(v2+10*sin(rightOris[k])), TColor::red );
            aux_rightImg.drawCircle( (int)u2, (int)v2, 2, TColor::red );
            mrpt::system::os::sprintf(buf, 10, "%d", (int)itMatches->second->ID );
            aux_rightImg.textOut( (int)u2, (int)v2, buf ,TColor::blue );
        }
    } // end for
    cout << "Time: " << ti*1000.0f/matchedFeats.size() << endl;
//    CDisplayWindow nw1, nw2;
//    nw1.showImage(aux_leftImg);
//    nw2.showImage(aux_rightImg);
//    mrpt::system::pause();
















        //if( oriR < 0 ) continue;
//        if( oriL == 0 && fabs(oriL-oriR) < DEG2RAD(5) )
//        {

//            imageLeft.line( (int)u1, (int)v1, (int)(u1+10*cos(oriL)), (int)(v1-10*sin(oriL)), TColor::red );
//            imageLeft.drawCircle( (int)u1, (int)v1, 2, TColor::red );
//            mrpt::system::os::sprintf(buf, 10, "%d", itMatches->first->ID );
//            imageLeft.textOut( (int)u1, (int)v1, buf ,TColor::blue );
//
//            imageRight.line( (int)u2, (int)v2, (int)(u2+10*cos(oriR)), (int)(v2-10*sin(oriR)), TColor::red );
//            imageRight.drawCircle( (int)u2, (int)v2, 2, TColor::red );
//            mrpt::system::os::sprintf(buf, 10, "%d", itMatches->second->ID );
//            imageRight.textOut( (int)u2, (int)v2, buf ,TColor::blue );



//            cout << "#" << counter << " - " << oriL << "," << oriR << "Dif: " << fabs(oriL-oriR) << endl;
//            goodmatches++;
//
//            // Equations taking into account that the cameras can be different
//            double f1,f2,c1,r1,c2,r2;
//            f1 = leftCamera.fx();
//            c1 = leftCamera.cx();
//            r1 = leftCamera.cy();
//            f2 = rightCamera.fx();
//            c2 = rightCamera.cx();
//            r2 = rightCamera.cy();
//
//            double b = rcTrans[0];  // Baseline
//
////            const double X = (u1-c1)*(f2*(b-c2+u2)+f1*(c2-u2))/(f2*(u1-c1)+f1*(c2-u2));
////            const double Y = (v1-r1)*(f2*(b-r2+v2)+f1*(r2-v2))/(f2*(v1-v1)+f1*(r2-v2));
////            const double Z = (f2*(b-c2+u2)+f1*(c2-u2))/(c2-u2+(f2/f1)*(u1-c1));
//
//            const double X = ( u1 - c1 ) * ( b ) / (u1-u2);
//			const double Y = ( v1 - r1 ) * ( b ) / (u1-u2);
//			const double Z = ( f1 ) * ( b ) / (u1-u2);
//
//            const double d = sqrt( X*X + Y*Y + Z*Z );
//            if( d < 0.5f ) continue;
//
////            //Compute the proper size of the patch (from 32px to 9px, for example) according to the distance
////            double smax = 32.0f;
////            double smin = 9.0f;
////            double ratio_s = smax/smin;
////            double dmin = 0.5f;
////            unsigned int n = floor( log(d/dmin)/log(ratio_s) ) + 1;
////            const unsigned int sift_patchSize = (unsigned int)( smax*pow(ratio_s,n-1)*dmin/d );
////            mrpt::system::os::fprintf( f, "%.4f %d\n", d, sift_patchSize );
//
//            // Patch computation:
//            // ---------------------------------------------------------
//            // [Important]
//            // Number of samples fixed: NdxNd = 21x21
//            // Patch size: p = Nd*k-k+1 where k:0.25:0.25:2.0 is the sampling step
//
//            // Compute the histogram of orientations in the patch + normalize + crop + normalize --> Descriptor
////            vector<vector<char>>descriptors( 8 );   // one for each value of k: 0.25:0.25:2.0
//            vector<int> descLeft(128);
//            vector<int> descRight(128);
//            const double crop_value = 0.2;
//            double k = 1;
//            unsigned int sift_patchSize = 21;
//            cout << "Computing Histogram of orientations (1/2)";
////            vision::computeHistogramOfOrientationsMulti(
////                imageLeft, u1, v1, sift_patchSize, oriL, descriptors, crop_value );
//            vision::computeHistogramOfOrientations(
//                imageLeft,                                  // the left image
//                u1, v1,                                     // the position of the keypoint
//                sift_patchSize,                             // the size of the patch
//                k,                                          // the step in resolution
//                oriL,                                       // the orientation that must be applied to the patch
//                descLeft,                                   // the OUTPUT descriptor
//                crop_value );                               // the value for cropping the descriptor before second normalization
//            cout << " ... done" << endl;
//            mrpt::system::os::fprintf( f, "%d ", itMatches->first->ID );
//            for( unsigned int ii = 0; ii < descLeft.size(); ++ii )
//                mrpt::system::os::fprintf( f, "%d ", descLeft[ii] );
//            mrpt::system::os::fprintf( f, "\n" );
//
//            CImage leftPatch, rightPatch;
//            imageLeft.extract_patch( leftPatch, u1-10, v1-10, 21, 21 );
//            imageRight.extract_patch( rightPatch, u2-10, v2-10, 21, 21 );
//            leftPatch.saveToFile("imgs/lpatch.jpg");
//            rightPatch.saveToFile("imgs/rpatch.jpg");
//            //cout << descriptor << endl;
//
//            vision::computeHistogramOfOrientations(
//                imageLeft,                                  // the left image
//                u1, v1,                                     // the position of the keypoint
//                sift_patchSize,                             // the size of the patch
//                k,                                          // the step in resolution
//                oriL,                                       // the orientation that must be applied to the patch
//                descLeft,                                   // the OUTPUT descriptor
//                crop_value );                               // the value for cropping the descriptor before second normalization
//            cout << " ... done" << endl;
//            mrpt::system::os::fprintf( f, "%d ", itMatches->first->ID );
//            for( unsigned int ii = 0; ii < descLeft.size(); ++ii )
//                mrpt::system::os::fprintf( f, "%d ", descLeft[ii] );
//            mrpt::system::os::fprintf( f, "\n" );
//
//    //            cout << "Computing Histogram of orientations (2/2)";
//    //            vision::computeHistogramOfOrientations(
//    //                imageRight,                                 // the right image
//    //                u2, v2,                                     // the position of the keypoint
//    //                sift_patchSize,                             // the size of the patch
//    //                k,                                          // the step in resolution
//    //                oriR,                                       // the orientation that must be applied to the patch
//    //                descRight,                                  // the OUTPUT descriptor
//    //                crop_value );                               // the value for cropping the descriptor before second normalization
//    //            cout << " ... done" << endl;
//    //            mrpt::system::os::fprintf( f, "%d ", itMatches->second->ID );
//    //            for( unsigned int ii = 0; ii < descRight.size(); ++ii )
//    //                mrpt::system::os::fprintf( f, "%d ", descRight[ii] );
//    //            mrpt::system::os::fprintf( f, "\n" );
//
//            if( goodmatches == 1 )
//            {
//                mrpt::system::os::fclose(f);
//                return;
//            }
//        } // end if
//    }
//    mrpt::system::os::fclose(f);
//
//    CDisplayWindow outWin1("Left Image"), outWin2("Right Image");
//    outWin1.showImage( imageLeft );
//    outWin2.showImage( imageRight );
//
//    mrpt::system::pause();
//
//    cout << "Good oriented matches: " << goodmatches;
////    cout << "Vector: " << endl;
////    for( unsigned int ii = 0; ii < this_ori.size(); ++ii )
////        cout << this_ori[ii] << ";";
////    cout << endl;
//
////    CDisplayWindow outWin1, outWin2;
////    outWin1.showImage( imageLeft );
////    outWin2.showImage( imageRight );
//
//    // Now compute the depth of the point and
//
//    mrpt::system::pause();

} // end-orientation.test

void multiResDesc_test()
{
    CTimeLogger tlogger;
    tlogger.disable();
    FILE *fl = mrpt::system::os::fopen( "imgs/dist.txt", "wt" );
    FILE *fr = mrpt::system::os::fopen( "imgs/right_desc.txt", "wt" );
    // Capture a pair of images (640x480) ??
    // Rectify them or capture them already rectified
    CImage imageLeft1, imageLeft2, imageRight1, imageRight2;

//	string imgL1 = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imL_p01.jpg"); //string("imgs/imL_p01_320.jpg");		// Left image
//	string imgR1 = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imR_p01.jpg"); //string("imgs/imR_p01_320.jpg");		// Right image

	string imgL1 = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/img_stereo_2_left_01068.jpg");       // Left image
	string imgL2 = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/img_stereo_2_left_01070.jpg");      // Right image

	string imgR1 = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/img_stereo_2_right_01069.jpg");       // Left image
	string imgR2 = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/img_stereo_2_right_01071.jpg");      // Right image

    imageLeft1.loadFromFile( imgL1 );
    imageLeft1.grayscaleInPlace();
    imageRight1.loadFromFile( imgR1 );
    imageRight1.grayscaleInPlace();

    imageLeft2.loadFromFile( imgL2 );
    imageLeft2.grayscaleInPlace();
    imageRight2.loadFromFile( imgR2 );
    imageRight2.grayscaleInPlace();

    // LOAD INTRINSIC AND DISTORTION PARAMS OF THE CAMERAS FROM CONFIG FILE
    TCamera leftCamera, rightCamera;
    leftCamera.loadFromConfigFile( "LEFT_CAMERA", *iniFile );
    rightCamera.loadFromConfigFile( "RIGHT_CAMERA", *iniFile );

    vector_double rcTrans(3);
    iniFile->read_vector( "RIGHT_CAMERA", "rcTrans", vector_double(), rcTrans, true );//

    // Find (FAST) features
    CFeatureExtraction fExt;
    fExt.options.loadFromConfigFile( *iniFile, "EXTRACTION" );
    fExt.options.dumpToConsole();

    CFeatureList leftFeats1, leftFeats2, rightFeats1, rightFeats2;      // Detect features
    fExt.detectFeatures( imageLeft1, leftFeats1 );
    fExt.detectFeatures( imageRight1, rightFeats1 );

    //fExt.options.FASTOptions.nonmax_suppression = false;
    fExt.detectFeatures( imageLeft2, leftFeats2 );
    fExt.detectFeatures( imageRight2, rightFeats2 );

    //**************************************************************************
    // Match the features
    //**************************************************************************
    CMatchedFeatureList             matchedFeats1, matchedFeats2;

    TMatchingOptions options;
    options.loadFromConfigFile( *iniFile, "MatchingOptions" );
    options.fx         = leftCamera.fx();
    options.cx         = leftCamera.cx();
    options.cy         = leftCamera.cy();
    options.baseline   = rcTrans[0];
    options.dumpToConsole();

    const unsigned int nMatches1 = vision::matchFeatures( leftFeats1, rightFeats1, matchedFeats1, options );
    const unsigned int nMatches2 = vision::matchFeatures( leftFeats2, rightFeats2, matchedFeats2, options );

    cout << "nMatches1 = " << nMatches1 << "/[" << leftFeats1.size() << "," << rightFeats1.size() << "], ";
    cout << "nMatches2 = " << nMatches2 << "/[" << leftFeats2.size() << "," << rightFeats2.size() << "]" << endl;

    CDisplayWindow wMatch1("Matches First"), wMatch2("Matches Second");
    wMatch1.showImagesAndMatchedPoints( imageLeft1, imageRight1, matchedFeats1, TColor::red, true );
    wMatch2.showImagesAndMatchedPoints( imageLeft2, imageRight2, matchedFeats2, TColor::red, true );
    mrpt::system::pause();

//    leftFeats2.copyListFrom( leftFeats1 );
//    rightFeats2.copyListFrom( rightFeats1 );
//    const unsigned int nMatches2 = vision::matchFeatures( leftFeats2, rightFeats2, matchedFeats2, options );

    // Save the patches:
//    CMatchedFeatureList::iterator itMatch;
//    unsigned int k;
//    for( k = 0, itMatch = matchedFeats1.begin(); itMatch != matchedFeats1.end(); ++itMatch, ++k )
//    {
//        itMatch->first->patch.saveToFile( format("imgs/out/feat1_left%d.jpg",k) );
//        itMatch->second->patch.saveToFile( format("imgs/out/feat1_right%d.jpg",k) );
//    }
//    for( k = 0, itMatch = matchedFeats2.begin(); itMatch != matchedFeats2.end(); ++itMatch, ++k )
//    {
//        itMatch->first->patch.saveToFile( format("imgs/out/feat2_left%d.jpg",k) );
//        itMatch->second->patch.saveToFile( format("imgs/out/feat2_right%d.jpg",k) );
//    }

//    CDisplayWindow w1("Left First"), w2("Second");
//    w1.showImageAndPoints( imageLeft1, leftFeats1, TColor::red );
//    w2.showImageAndPoints( imageRight1, rightFeats1, TColor::red );
//    mrpt::system::pause();

    //--------------------------------------------------------------------------
    TMultiResDescOptions opts1, opts2;
    opts1.loadFromConfigFile( *iniFile, "MultiDescOptions1" );
    opts1.dumpToConsole();

    opts2.loadFromConfigFile( *iniFile, "MultiDescOptions2" );
    opts2.dumpToConsole();

    CFeatureList nlist1, nlist2, nnlist1, nnlist2;
    matchedFeats1.getBothFeatureLists( nlist1, nlist2 );
    matchedFeats2.getBothFeatureLists( nnlist1, nnlist2 );

    tlogger.enter("compute descriptors");
    vision::computeMultiResolutionDescriptors( imageLeft1, nlist1, opts1 );
    tlogger.leave("compute descriptors");

    // Matching
    TMultiResDescMatchOptions mrdMatchingOptions;
    mrdMatchingOptions.loadFromConfigFile( *iniFile, "MatchMultiDescOptions" );
    mrdMatchingOptions.dumpToConsole();

    // FIND MATCHES!!
    vector<int> leftMatchingIdx, rightMatchingIdx;
    CTicTac tictac;
    tictac.Tic();

//    vision::matchMultiResolutionFeatures(
//        leftFeats1, leftFeats2, imageLeft2,
//        leftMatchingIdx, rightMatchingIdx, mrdMatchingOptions, opts1 );
    vision::matchMultiResolutionFeatures(
        nlist1, nnlist1, imageLeft2,
        leftMatchingIdx, rightMatchingIdx, mrdMatchingOptions, opts1 );

    double nmtt = tictac.Tac();
    size_t nm = leftMatchingIdx.size();
    cout << "Found " << nm << " inter-frame matches in " << nmtt*1000.0f << " ms." << endl;

    // show them in the images:
    vector<float> x1(nm),x2(nm),y1(nm),y2(nm);
    for( int k = 0; k < nm; ++k )
    {
        x1[k] = nlist1[leftMatchingIdx[k]]->x;
        y1[k] = nlist1[leftMatchingIdx[k]]->y;
        x2[k] = nnlist1[rightMatchingIdx[k]]->x;
        y2[k] = nnlist1[rightMatchingIdx[k]]->y;
    } //

    CDisplayWindow newWindow1("Inter-frame I matches");
    CDisplayWindow newWindow2("Inter-frame I+1 matches");
    newWindow1.showImageAndPoints( imageLeft1, x1, y1, TColor::red, true );
    int hw = mrdMatchingOptions.searchAreaSize/2;
    for( int k = 0; k < nm; ++k )
    {
        imageLeft2.rectangle( nlist1[leftMatchingIdx[k]]->x-hw, nlist1[leftMatchingIdx[k]]->y-hw,
            nlist1[leftMatchingIdx[k]]->x+hw, nlist1[leftMatchingIdx[k]]->y+hw, TColor::red );
    }
    newWindow2.showImageAndPoints( imageLeft2, x2, y2, TColor::red, true );
    mrpt::system::pause();
    return;

    CFeatureList list1, rub, list2;
    matchedFeats1.getBothFeatureLists( list1, rub );
    matchedFeats2.getBothFeatureLists( list2, rub );

    tlogger.enter("compute descriptors 2");
    vision::computeMultiResolutionDescriptors( imageLeft2, imageRight2, matchedFeats2, opts2 );
    //vision::computeMultiResolutionDescriptors( imageLeft1, imageRight1, matchedFeats2, opts2 );
    tlogger.leave("compute descriptors 2");

    CFeatureList leftList1, rightList1, leftList2, rightList2;
    matchedFeats1.getBothFeatureLists( leftList1, rightList1 );
    matchedFeats2.getBothFeatureLists( leftList2, rightList2 );

//    tlogger.enter("compute descriptors 1");
//    vision::computeMultiResolutionDescriptors(
//        imageLeft1, imageRight1,
//        matchedFeats1,
//        leftMultiFeats1, rightMultiFeats1,
//        opts1 );
//    tlogger.leave("compute descriptors 1");
//
//    tlogger.enter("compute descriptors 2");
//    vision::computeMultiResolutionDescriptors(
//        imageLeft1, imageRight1,
//        matchedFeats1,
//        leftMultiFeats2, rightMultiFeats2,
//        opts2 );
//    tlogger.leave("compute descriptors 2");

    // STEREO MATCHING
    TMultiResDescOptions opts3;
    opts3.loadFromConfigFile( *iniFile, "MultiDescOptions3" );
//    opts3.dumpToTextStream( outStream );

//    opts3.useDepthFilter = true;
//    opts3.matchingThreshold = 1e4;
//    opts3.matchingRatioThreshold = 1.0;
    tlogger.enter("Force brute matching");

    vector<int> matches_idx;
    vector<double> matches_scales_left, matches_orientation_left, matches_scales_right, matches_orientation_right;
    vector<double> dist_corrs;

//    vision::matchMultiResolutionFeatures(
//        leftMultiFeats1, rightMultiFeats2,
//        matches_idx, dist_corrs,
//        matches_scales_left, matches_scales_right,
//        matches_orientation_left, matches_orientation_right,
//        opts3 );

    vision::matchMultiResolutionFeatures(
        leftList1, leftList2,
        matches_idx, dist_corrs,
        matches_scales_left, matches_scales_right,
        matches_orientation_left, matches_orientation_right, mrdMatchingOptions );
    tlogger.leave("Force brute matching");

    // Generate a view of the matched features.
    size_t aux_size = matches_idx.size();
    vector_float u1,v1,u2,v2;
    unsigned int cnt = 0, gcnt = 0;
    for( unsigned int k = 0; k < aux_size; ++k )
    {
        if( matches_idx[k] > 0 )
        {
            u1.push_back( leftList2[k]->x );
            v1.push_back( leftList2[k]->y );
            u2.push_back( leftList1[matches_idx[k]]->x );
            v2.push_back( leftList1[matches_idx[k]]->y );
            cnt++;
            if( k == matches_idx[k] )
                gcnt++;
        }
    }
    cout << "matching completed" << endl << matches_idx << endl;
    cout << "nmatches: " << cnt << "/" << matches_idx.size() << " (" << gcnt << " good)" << endl;
//    CDisplayWindow newWindow1("Inter-frame Left matches");
//    CDisplayWindow newWindow2("Inter-frame Right matches");
//    newWindow1.showImageAndPoints( imageLeft2, u1, v1, TColor::red, true );
//    newWindow2.showImageAndPoints( imageLeft1, u2, v2, TColor::red, true );
//    cout << "MATCHES: " << matches_idx.size() << endl;
//
//    for( unsigned int k = 0; k < matches_idx0.size(); ++k )
//    {
//        mrpt::system::os::fprintf( fl, "%d %d %d %d %d %d\n",
//                                matches_idx0[k].first, matches_idx0[k].second,
//                                matches_scale_idx0[k].first,  matches_scale_idx0[k].second,
//                                matches_orientation_idx0[k].first, matches_orientation_idx0[k].second );
//
//        cout << "L:" << matches_idx0[k].first << " R:" << matches_idx0[k].second;
//        cout << " lscl:" << matches_scale_idx0[k].first << " rscl:" << matches_scale_idx0[k].second;
//        cout << " lori:" << matches_orientation_idx0[k].first << " rori:" << matches_orientation_idx0[k].second;
//        cout << " dist:" << matches_distance0[k] << endl;
//    }
    mrpt::system::os::fclose( fl );
//    mrpt::system::pause();
//    vision::matchMultiResolutionFeatures(
//            leftMultiFeats1, leftMultiFeats2,
//            matches_idx1,
//            matches_scale_idx1,
//            matches_orientation_idx1,
//            matches_distance1,
//            opts );
//
//    vision::matchMultiResolutionFeatures(
//            rightMultiFeats1, rightMultiFeats2,
//            matches_idx2,
//            matches_scale_idx2,
//            matches_orientation_idx2,
//            matches_distance2,
//            opts );



//    cout << "TOTAL MATCHES: " << matches_idx.size() << " of " << good_matches << " in " << tmatching*1000.0f << " ms" << endl;
//    for( unsigned int k = 0; k < matches_idx1.size(); ++k )
//    {
//        mrpt::system::os::fprintf( fl, "%d %d %d %d %d %d\n",
//                                matches_idx1[k].first, matches_idx1[k].second,
//                                matches_scale_idx1[k].first,  matches_scale_idx1[k].second,
//                                matches_orientation_idx1[k].first, matches_orientation_idx1[k].second );
//
//        cout << "L:" << matches_idx1[k].first << " R:" << matches_idx1[k].second;
//        cout << " lscl:" << matches_scale_idx1[k].first << " rscl:" << matches_scale_idx1[k].second;
//        cout << " lori:" << matches_orientation_idx1[k].first << " rori:" << matches_orientation_idx1[k].second;
//        cout << " dist:" << matches_distance1[k] << endl;
//    }
//    mrpt::system::os::fclose( fl );
//    for( unsigned int k = 0; k < matches_idx1.size(); ++k )
//    {
//        mrpt::system::os::fprintf( fr, "%d %d %d %d %d %d\n",
//                                matches_idx1[k].first, matches_idx1[k].second,
//                                matches_scale_idx1[k].first,  matches_scale_idx1[k].second,
//                                matches_orientation_idx1[k].first, matches_orientation_idx1[k].second );
//
//        cout << "L:" << matches_idx1[k].first << " R:" << matches_idx1[k].second;
//        cout << " lscl:" << matches_scale_idx1[k].first << " rscl:" << matches_scale_idx1[k].second;
//        cout << " lori:" << matches_orientation_idx1[k].first << " rori:" << matches_orientation_idx1[k].second;
//        cout << " dist:" << matches_distance1[k] << endl;
//    }
//    mrpt::system::os::fclose( fr );

        // For each match, resize the patch to 23x23
        // Compute main orientation within the patch (some modifications have to be done here)
        // For each of the detected orientations:
            // Rotate the w x w (w = 16) patch and compute the new histogram of orientations (smooth the histogram with a Gaussian sigma = w/2)
            // Descriptor: Bp x Bp x Bn = 4 x 4 x 8
            // Use normalization + truncate + normalization for the descriptor

//    fExt.options.patchSize = 0;
//
//    CFeatureList leftFeats,rightFeats;
//    fExt.detectFeatures( imageLeft, leftFeats );
//    fExt.detectFeatures( imageRight, rightFeats );
//
//    // Match them [~85 matches]
//    CMatchedFeatureList matchedFeats;
//    TMatchingOptions options;
//
//    options.epipolarTH = 3.0;           // Load from config file??
//    unsigned int nMatches = vision::matchFeatures( leftFeats, rightFeats, matchedFeats );
//    cout << "# matches: " << nMatches << endl;
//
//    // Compute the 3D distance from the camera
//    TCamera leftCamera, rightCamera;
//    leftCamera.loadFromConfigFile( *iniFile, "LEFT_CAMERA" );
//    rightCamera.loadFromConfigFile( *iniFile, "RIGHT_CAMERA" );
//
//    CMatchedFeatureList::iterator itMatches;
//    for( itMatches = matchedFeats.begin(); itMatches != matchedFeats.end(); ++itMatches )
//    {
//        // Equations taking into account that the cameras can be different
//        const double X = ;
//        const double Y = ;
//        const double Z = ;
//
//        const double d = sqrt( X*X + Y*Y + Z*Z );
//
//        // Compute the proper size of the patch (from 32px to 9px, for example) according to the distance
//        const unsigned int patchSize = (unsigned int)( smax*pow(smax/smin,n-1)*dmin/d) );
//
//        // Compute the main orientation and rotate the patch according to it
//        const double mOriLeft   = vision::computeMainOrientation(
//                                    imageLeft,
//                                    itMatches->first->x, itMatches->first->y,
//                                    patchSize );
//
//        const double mOriRight  = vision::computeMainOrientation(
//                                    imageRight,
//                                    itMatches->second->x, itMatches->second->y,
//                                    patchSize )
//
//        // Compute the histogram of orientations in the patch + normalize + crop + normalize --> Descriptor
//        vector<char> descriptor(128);
//        const double crop_value = 0.2;
//        vision::computeHistogramOfOrientations(
//            leftImage,                                  // the left image
//            itMatches->first->x, itMatches->first->y,   // the position of the keypoint
//            patchSize,                                  // the size of the patch
//            mOriLeft,                                   // the orientation that must be applied to the patch
//            descriptor,                                 // the OUTPUT descriptor
//            crop_value );                               // the value for cropping the descriptor before second normalization
//
//        vision::computeHistogramOfOrientations(
//            rightImage,                                 // the right image
//            itMatches->second->x, itMatches->second->y, // the position of the keypoint
//            patchSize,                                  // the size of the patch
//            mOriRight,                                  // the orientation that must be applied to the patch
//            descriptor,                                 // the OUTPUT descriptor
//            crop_value );                               // the value for cropping the descriptor before second normalization
//
//    } // end-for-itMatches
} // end-multiResDesc_test()

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
//    mrpt::slam::CLandmarksMap landmarks;
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
				if( !counter )
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
		ASSERT_(fileExists(INI_FILENAME));

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
		// vOdometry_lightweight();
		//vOdometry_onthefly();
		//orientation_test();
		multiResDesc_test();
		//detectFASTinSequence();

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

