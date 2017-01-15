/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/maps/CLandmarksMap.h>
#include <mrpt/gui/CDisplayWindow.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>

using mrpt::format;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::vision;
using namespace mrpt::opengl;
using namespace std;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("keypoint_matching/imgs/") );

// ------------------------------------------------------
//	TestExtractMatchProjectAndPaint
// ------------------------------------------------------
void TestExtractMatchProjectAndPaint()
{
	CFeatureExtraction	fExt;
	CFeatureList		featsHarris_L, featsHarris_R;
	CMatchedFeatureList	mHarris, mSIFT, mSURF;
	CImage				imL, imR;

	string imgL = myDataDir + string("imL_p01.jpg");		// Left image
	string imgR = myDataDir + string("imR_p01.jpg");		// Right image

	// Load and check images
	if (!imL.loadFromFile( imgL ))
	{
		cerr << "Cannot load " << imgL  << endl;
		return;
	}
	cout << "Loaded test image: " << imgL << endl;

	if (!imR.loadFromFile( imgR ))
	{
		cerr << "Cannot load " << imgR  << endl;
		return;
	}
	cout << "Loaded test image: " << imgR << endl;

	cout << "***************************************************" << endl;
	cout << "***************************************************" << endl;

	// Extract features:
	// HARRIS
	cout << "Detecting HARRIS features in LEFT image" << endl;
	fExt.options.featsType = featKLT;
	fExt.detectFeatures( imL, featsHarris_L );
	cout << "Detected " << featsHarris_L.size() << endl;

	cout << "Detecting HARRIS features in RIGHT image" << endl;
	fExt.detectFeatures( imR, featsHarris_R );
	cout << "Detected " << featsHarris_R.size() << endl;

	cout << "***************************************************" << endl;
	cout << "***************************************************" << endl;

	// Match features:
	//size_t nMatches;
	TMatchingOptions opt;
	cout << "Matching HARRIS features by CORRELATION" << endl;
	//nMatches =
	matchFeatures( featsHarris_L, featsHarris_R, mHarris );
	cout << "Matches found: " << mHarris.size() << endl;

	cout << "***************************************************" << endl;

	// Project features:
	mrpt::maps::CLandmarksMap   outMap;
	TStereoSystemParams         stereoOptions;                      // Default options: Bumblebee + 640x480
	cout << "Projecting matched features" << endl;
	mrpt::vision::projectMatchedFeatures( mHarris, stereoOptions, outMap );

    CDisplayWindow3D    win3D("3D Map");
    COpenGLScenePtr		&scene3D    = win3D.get3DSceneAndLock();
	CSetOfObjectsPtr    map3D       = CSetOfObjects::Create();
	outMap.getAs3DObject( map3D );
    CGridPlaneXYPtr	gridXY	        = CGridPlaneXY::Create(-10,10,-10,10,0,1);

	scene3D->insert( gridXY );
	scene3D->insert( map3D );

    win3D.unlockAccess3DScene();
	win3D.repaint();

	mrpt::system::pause();

} // end TestExtractMatchProjectAndPaint

// ------------------------------------------------------
//		TestMatchFeatures
// ------------------------------------------------------
void TestMatchFeatures( bool showMatches )
{

	CFeatureExtraction	fExt;
	CFeatureList		featsHarris_L, featsHarris_R, featsSIFT_L, featsSIFT_R, featsSURF_L, featsSURF_R, featsFAST_L, featsFAST_R;
	CMatchedFeatureList	mHarris, mSIFT, mSURF, mHarris_SAD, mFAST_CC, mFAST_SAD;
	CImage				imL, imR;

	string imgL = myDataDir + string("imL_p01.jpg");		// Left image
	string imgR = myDataDir + string("imR_p01.jpg");		// Right image

	// Load and check images
	if (!imL.loadFromFile( imgL ))
	{
		cerr << "Cannot load " << imgL  << endl;
		return;
	}
	cout << "Loaded LEFT image: " << endl << imgL << endl;

	if (!imR.loadFromFile( imgR ))
	{
		cerr << "Cannot load " << imgR  << endl;
		return;
	}
	cout << "Loaded RIGHT image: " << endl << imgR << endl;

	cout << "***************************************************" << endl;
	cout << "***************************************************" << endl;

	// Extract features:
    // HARRIS
	cout << "Detecting HARRIS features in LEFT image" << endl;
	fExt.options.featsType = featHarris;
	fExt.detectFeatures( imL, featsHarris_L, 250 );
	cout << "Detected " << featsHarris_L.size() << endl;

	cout << "Detecting HARRIS features in RIGHT image" << endl;
	fExt.detectFeatures( imR, featsHarris_R, 250 );
	cout << "Detected " << featsHarris_R.size() << endl;
	cout << "***************************************************" << endl;

	// SIFT
	cout << "Detecting SIFT features in LEFT image" << endl;
	fExt.options.featsType = featSIFT;
	//fExt.options.SIFTOptions.implementation = CFeatureExtraction::Hess;
	fExt.options.SIFTOptions.implementation = CFeatureExtraction::OpenCV;
	fExt.detectFeatures( imL, featsSIFT_L );
	cout << "Detected " << featsSIFT_L.size() << endl;

	cout << "Detecting SIFT features in RIGHT image" << endl;
	fExt.options.featsType = featSIFT;
	//fExt.options.SIFTOptions.implementation = CFeatureExtraction::Hess;
	fExt.options.SIFTOptions.implementation = CFeatureExtraction::OpenCV;
	fExt.detectFeatures( imR, featsSIFT_R );
	cout << "Detected " << featsSIFT_R.size() << endl;
	cout << "***************************************************" << endl;

	// SURF
	cout << "Detecting SURF features in LEFT image" << endl;
	fExt.options.featsType = featSURF;
	fExt.detectFeatures( imL, featsSURF_L );
	cout << "Detected " << featsSURF_L.size() << endl;

	cout << "Detecting SURF features in RIGHT image" << endl;
	fExt.detectFeatures( imR, featsSURF_R );
	cout << "Detected " << featsSURF_R.size() << endl;
	cout << "***************************************************" << endl;

	// FAST
	cout << "Detecting FAST features in LEFT image" << endl;
	fExt.options.featsType = featFAST;
	fExt.detectFeatures( imL, featsFAST_L, 0, 250 );
	cout << "Detected " << featsFAST_L.size() << endl;

	cout << "Detecting FAST features in RIGHT image" << endl;
	fExt.detectFeatures( imR, featsFAST_R, 0, 250 );
	cout << "Detected " << featsFAST_R.size() << endl;
	cout << "***************************************************" << endl;
	cout << "***************************************************" << endl;

	// Match features:
	size_t nMatches;
	TMatchingOptions opt;

//	// HARRIS
	CTicTac tictac;
	double T = 0.0;
	cout << "Matching HARRIS features" << endl;
	tictac.Tic();
	nMatches = matchFeatures( featsHarris_L, featsHarris_R, mHarris );
	T = tictac.Tac();
	cout << "[NCC] Matches found: " << mHarris.size() << " in " << T*1000.0f << " ms " << endl;

	opt.matching_method = TMatchingOptions::mmSAD;
	tictac.Tic();
	nMatches = matchFeatures( featsHarris_L, featsHarris_R, mHarris_SAD, opt );
	T = tictac.Tac();
	cout << "[SAD] Matches found: " << mHarris_SAD.size() << " in " << T*1000.0f << " ms " << endl;
	cout << "***************************************************" << endl;

	// SIFT
	cout << "Matching SIFT features by DESCRIPTOR" << endl;
	opt.matching_method = TMatchingOptions::mmDescriptorSIFT;
	tictac.Tic();
	nMatches = matchFeatures( featsSIFT_L, featsSIFT_R, mSIFT, opt );
	T = tictac.Tac();
	cout << "Matches found: " << mSIFT.size() << " in " << T*1000.0f << " ms " << endl;
	cout << "***************************************************" << endl;

	// SURF
	cout << "Matching SURF features by DESCRIPTOR" << endl;
	opt.matching_method = TMatchingOptions::mmDescriptorSURF;
	tictac.Tic();
	nMatches = matchFeatures( featsSURF_L, featsSURF_R, mSURF, opt );
	T = tictac.Tac();
	cout << "Matches found: " << mSURF.size() << " in " << T*1000.0f << " ms " << endl;
	cout << "***************************************************" << endl;

	// FAST
	cout << "Matching FAST features" << endl;
	tictac.Tic();
	nMatches = matchFeatures( featsFAST_L, featsFAST_R, mFAST_CC );
	T = tictac.Tac();
	cout << "[NCC] Matches found: " << mFAST_CC.size() << " in " << T*1000.0f << " ms " << endl;

	opt.matching_method = TMatchingOptions::mmSAD;
	tictac.Tic();
	nMatches = matchFeatures( featsFAST_L, featsFAST_R, mFAST_SAD, opt );
	T = tictac.Tac();
	cout << "[SAD] Matches found: " << mFAST_SAD.size() << " in " << T*1000.0f << " ms " << endl;
	cout << "***************************************************" << endl;


    if( showMatches )
    {
        CDisplayWindow winHarrisSAD, winHarrisNCC, winFASTSAD, winFASTNCC, winSIFT, winSURF;

        winHarrisSAD.setWindowTitle("Matches with Harris + SAD");
        winHarrisNCC.setWindowTitle("Matches with Harris + NCC");
        winFASTSAD.setWindowTitle("Matches with FAST + SAD");
        winFASTNCC.setWindowTitle("Matches with FAST + NCC");
        winSIFT.setWindowTitle("Matches with SIFT");
        winSURF.setWindowTitle("Matches with SURF");

        winHarrisNCC.showImagesAndMatchedPoints( imL, imR, mHarris, TColor(0,0,255) );
        winHarrisSAD.showImagesAndMatchedPoints( imL, imR, mHarris_SAD, TColor(0,0,255) );
        winSIFT.showImagesAndMatchedPoints( imL, imR, mSIFT, TColor(0,255,0) );
        winSURF.showImagesAndMatchedPoints( imL, imR, mSURF, TColor(0,255,0) );
        winFASTSAD.showImagesAndMatchedPoints( imL, imR, mFAST_SAD, TColor(0,255,0) );
        winFASTNCC.showImagesAndMatchedPoints( imL, imR, mFAST_CC, TColor(0,255,0) );

        mrpt::system::pause();
    }

} // end TestMatchFeatures

// ------------------------------------------------------
//		TestMatchingComparative
// ------------------------------------------------------
void TestMatchingComparative()
{
    // Take two images
    string imgL = myDataDir + string("imL_p01.jpg");		// Left image
	string imgR = myDataDir + string("imR_p01.jpg");		// Right image

    CImage im1, im2;
    im1.loadFromFile( imgL );
    im2.loadFromFile( imgR );

    size_t imW = im1.getWidth();
    size_t imH = im1.getHeight();

    CFeatureExtraction fExt;
    fExt.options.featsType                  = featFAST;
    fExt.options.patchSize                  = 21;
    fExt.options.SIFTOptions.implementation = CFeatureExtraction::Hess;

    // Find FAST features
    CFeatureList list1, list2;
    fExt.detectFeatures( im1, list1, 150 );
    // Compute SIFT & SURF descriptors
    fExt.computeDescriptors( im1, list1, descSIFT );
    fExt.computeDescriptors( im1, list1, descSURF );

    fExt.detectFeatures( im2, list2, 150 );
    // Compute SIFT & SURF descriptors
    fExt.computeDescriptors( im2, list2, descSIFT );
    fExt.computeDescriptors( im2, list2, descSURF );

    CFeatureList::iterator it1, it2;
    for( it1 = list1.begin(); it1 != list1.end(); ++it1 )
        im1.cross( (*it1)->x, (*it1)->y, TColor::red, '+');
    for( it2 = list2.begin(); it2 != list2.end(); ++it2 )
        im2.cross( (*it2)->x, (*it2)->y, TColor::red, '+');

    CDisplayWindow win, win2;
    win.setPos(0,0);
    win2.setPos(0,imH*1.5);
    CImage joinimage, copyjoinimage, copyInfoImage;
    size_t imW2 = 1280;
    size_t imH2 = 150;

    CImage infoimage( imW2, imH2, CH_RGB );

    joinimage.joinImagesHorz( im1, im2 );
    infoimage.filledRectangle( 0, 0, imW2, imH2, TColor(150,150,150) );
    infoimage.textOut( 20, imH2-53, "SAD", TColor::blue );
    infoimage.textOut( 20, imH2-41, "NCC", TColor::blue );
    infoimage.textOut( 20, imH2-29, "SIFT", TColor::blue );
    infoimage.textOut( 20, imH2-17, "SURF", TColor::blue );
    for( it1 = list1.begin(); it1 != list1.end(); ++it1 )
    {
        copyInfoImage = infoimage;
        copyjoinimage = joinimage;
        copyjoinimage.line( (*it1)->x, 0, (*it1)->x, imH, TColor::green );            // Horiz
        copyjoinimage.line( (*it1)->x+imW, 0, (*it1)->x+imW, imH, TColor::green );    // Horiz
        copyjoinimage.line( 0, (*it1)->y, imW+imW, (*it1)->y, TColor::green );        // Epipolar
        copyjoinimage.drawCircle( (*it1)->x, (*it1)->y, 4, TColor::green, 2 );        // Keypoint

        copyInfoImage.update_patch( (*it1)->patch, 0, 0 );
        bool firstMatch = true;
        int cnt = 0;
        int px = 80;
        double minsad = 1.0, maxncc = 0.0;
        float minsiftd = 1.0f, minsurfd = 1.0f;
        int idxsad = 0, idxncc = 0, idxsiftd = 0, idxsurfd = 0;

        for( it2 = list2.begin(); it2 != list2.end(); ++it2 )
        {
            if( fabs((*it1)->y-(*it2)->y) <= 1.0 && (*it1)->x > (*it2)->x )
            {
                    // Compute matching with SAD and Correlation and SIFT/SURF?
                    // Use epipolar constraints
                    // Compute SAD
                    double sad = mrpt::vision::computeSAD( (*it1)->patch, (*it2)->patch );
                    if( sad < minsad )
                    {
                        minsad = sad;
                        idxsad = cnt;
                    }
                    // Compute Correlation
                    double ncc;
                    size_t u, v;
                    mrpt::vision::openCV_cross_correlation( (*it1)->patch, (*it2)->patch, u, v, ncc );
                    if( ncc > maxncc )
                    {
                        maxncc = ncc;
                        idxncc = cnt;
                    }

                    // Compute distance between descriptors SIFT
                    float siftd = (*it1)->descriptorSIFTDistanceTo( *(*it2) );
                    if( siftd < minsiftd )
                    {
                        minsiftd = siftd;
                        idxsiftd = cnt;
                    }

                    // Compute distance between descriptors SIFT
                    float surfd = (*it1)->descriptorSURFDistanceTo( *(*it2) );
                    if( surfd < minsurfd )
                    {
                        minsurfd = surfd;
                        idxsurfd = cnt;
                    }

                    // Plot images + features + each candidate + difference score
                    if( firstMatch )
                    {
                        copyjoinimage.line( (*it1)->x+imW, 0, (*it1)->x+imW, imH, TColor::green );  // Limit line (only the first time)
                        firstMatch = false;
                    } // end-if

                    copyjoinimage.drawCircle( (*it2)->x+imW, (*it2)->y, 4, TColor::blue, 2 );       // Keypoint
                    double rx0, rx1, ry0, ry1, tx, ty;
                    rx0 = (*it2)->x+imW-15;
                    rx1 = (*it2)->x+imW;
                    tx = (*it2)->x+imW-13;
                    if( cnt % 2 )
                    {
                        ry0 = (*it2)->y-20;
                        ry1 = (*it2)->y-10;
                        ty = (*it2)->y-22;
                    }
                    else
                    {
                        ry0 = (*it2)->y+10;
                        ry1 = (*it2)->y+20;
                        ty = (*it2)->y+8;
                    }
                    copyjoinimage.filledRectangle( rx0, ry0, rx1, ry1, TColor(150,150,150) );
                    copyjoinimage.textOut( tx, ty, format("%d", cnt), TColor::blue );

                    px = 80+cnt*50;
                    if( px + fExt.options.patchSize > imW2 )
                        continue;

                    copyInfoImage.update_patch( (*it2)->patch, px, 30 );

                    copyInfoImage.textOut( px, imH2-70, format("%d", cnt), TColor::blue );
                    copyInfoImage.textOut( px, imH2-53, format("%.2f", sad), TColor::blue );
                    copyInfoImage.textOut( px, imH2-41, format("%.2f", ncc), TColor::blue );
                    copyInfoImage.textOut( px, imH2-29, format("%.2f", siftd), TColor::blue );
                    copyInfoImage.textOut( px, imH2-17, format("%.2f", surfd), TColor::blue );

                    cnt++;
            } // end if
        } // end for it2
        copyInfoImage.textOut( 80+idxsad*50, imH2-53, format("%.2f", minsad), TColor::green );
        copyInfoImage.textOut( 80+idxncc*50, imH2-41, format("%.2f", maxncc), TColor::green );
        copyInfoImage.textOut( 80+idxsiftd*50, imH2-29, format("%.2f", minsiftd), TColor::green );
        copyInfoImage.textOut( 80+idxsurfd*50, imH2-17, format("%.2f", minsurfd), TColor::green );

        win.showImage( copyjoinimage );
        win2.showImage( copyInfoImage );
        mrpt::system::pause();
    } // end for it1

    // Save to file
    // Check number of good features

} // end TestMatchingComparative

int main(int argc, char **argv)
{
	try
	{
	    if( argc == 1 )
	    {
	        cerr << "Usage: " << argv[0] << endl;
	        cerr << "Options:" << endl;
	        cerr << " -match [-s]: TestMatchFeatures (if -s is set, final matches in images will be shown)." << endl;
	        cerr << " -comp: TestMatchingComparative." << endl;
	        cerr << " -proj: TestExtractMatchProjectAndPaint." << endl;
	    }
	    else
	    {
            if(!strcmp(argv[1],"-match"))
            {
                if(argc == 3 && !strcmp(argv[2],"-s"))
                    TestMatchFeatures( true );
                else
                    TestMatchFeatures( false );
            }
            else if(!strcmp(argv[1],"-proj"))
                TestExtractMatchProjectAndPaint();
            else if(!strcmp(argv[1],"-comp"))
            {
                cout << "Press ^C to finish program." << endl;
                TestMatchingComparative();
            }
            else
            {
                cerr << "Usage: " << argv[0] << endl;
                cerr << "Options:" << endl;
                cerr << " -match [-s]: TestMatchFeatures (if -s is set, final matches in images will be shown)." << endl;
                cerr << " -comp: TestMatchingComparative." << endl;
                cerr << " -proj: TestExtractMatchProjectAndPaint." << endl;
            }
        }
		return 0;
	} catch (std::exception &e)
	{
		std::cout << "MRPT exception caught: " << e.what() << std::endl;
		return -1;
	}
	catch (...)
	{
		printf("Another exception!!");
		return -1;
	}

}
