/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/vision/CFeatureExtraction.h>
#include <mrpt/vision/tracking.h>
#include <mrpt/gui.h>

using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::vision;
using namespace std;
using mrpt::format;

#include <mrpt/examples_config.h>
string   myDataDir( MRPT_EXAMPLES_BASE_DIRECTORY + string("imageConvolutionFFT/") );
const string the_img_for_extract_feats = myDataDir+string("test_image.jpg");


void TestTrackFeatures()
{

	CImage im1, im2;
	im1.loadFromFile("/Trabajo/Experimentos/[2009] vOdometry Characterization/right1.jpg");
	im2.loadFromFile("/Trabajo/Experimentos/[2009] vOdometry Characterization/right2.jpg");

	CFeatureExtraction	fExt;
	CFeatureList		feats;

	fExt.options.featsType = featKLT;
	fExt.detectFeatures( im1, feats );
	feats.saveToTextFile("J:/Trabajo/Experimentos/[2009] vOdometry Characterization/before.txt");

	CFeatureTracker_KL	tracker;
	// tracker.extra_params["add_new_features"]  = 1;   // track, AND ALSO, add new features
	// ...

	// Do tracking:
	tracker.trackFeatures(im1, im2, feats);

	feats.saveToTextFile("/Trabajo/Experimentos/[2009] vOdometry Characterization/after.txt");
}

// ------------------------------------------------------
//				TestCapture
// ------------------------------------------------------
void TestExtractMatchProjectAndPaint()
{
	CDisplayWindow3D	wind;
	CFeatureExtraction	fExt;
	CFeatureList		featsHarris_L, featsHarris_R;
	CMatchedFeatureList	mHarris, mSIFT, mSURF;
	CImage				imL, imR;

	string imgL = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imL_p01.jpg");		// Left image
	string imgR = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imR_p01.jpg");		// Right image

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

	// HARRIS
	cout << "Matching HARRIS features by CORRELATION" << endl;
	//nMatches =
	matchFeatures( featsHarris_L, featsHarris_R, mHarris );
	cout << "Matches found: " << mHarris.size() << endl;

	cout << "***************************************************" << endl;

} // end TestExtractMatchProjectAndPaint

// ------------------------------------------------------
//				TestCapture
// ------------------------------------------------------
void TestMatchFeatures()
{
	CDisplayWindow		wind, wind2;
	CFeatureExtraction	fExt;
	CFeatureList		featsHarris_L, featsHarris_R, featsSIFT_L, featsSIFT_R, featsSURF_L, featsSURF_R, featsFAST_L, featsFAST_R;
	CMatchedFeatureList	mHarris, mSIFT, mSURF, mHarris_SAD, mFAST_CC, mFAST_SAD;
	CImage				imL, imR;

	string imgL = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imL_p01.jpg");		// Left image
	string imgR = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imR_p01.jpg");		// Right image

//	string imgL = "../../bin/imgs/640x480_left_rect.jpg";		// Left image
//	string imgR = "../../bin/imgs/640x480_right_rect.jpg";		// Right image

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
	fExt.options.featsType = featHarris;
	fExt.detectFeatures( imL, featsHarris_L );
	cout << "Detected " << featsHarris_L.size() << endl;

	cout << "Detecting HARRIS features in RIGHT image" << endl;
	fExt.detectFeatures( imR, featsHarris_R );
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
	fExt.detectFeatures( imL, featsFAST_L, 0, 400 );
	cout << "Detected " << featsFAST_L.size() << endl;
	CDisplayWindow fast1("LEFT");
	fast1.showImageAndPoints( imL, featsFAST_L );

	cout << "Detecting FAST features in RIGHT image" << endl;
	fExt.detectFeatures( imR, featsFAST_R, 0, 400 );
	cout << "Detected " << featsFAST_R.size() << endl;
	cout << "***************************************************" << endl;
	cout << "***************************************************" << endl;
	CDisplayWindow fast2("RIGHT");
	fast2.showImageAndPoints( imR, featsFAST_R );

	// Match features:
	//size_t nMatches;
	TMatchingOptions opt;

	// HARRIS
	CTicTac tictac;
	cout << "Matching HARRIS features by CORRELATION" << endl;
	tictac.Tic();
	//nMatches =
	matchFeatures( featsHarris_L, featsHarris_R, mHarris );
	double T = tictac.Tac();
	cout << "[CC] Matches found: " << mHarris.size() << " in " << T*1000.0f << " ms " << endl;

	opt.matching_method = TMatchingOptions::mmSAD;
	tictac.Tic();
	//nMatches =
	matchFeatures( featsHarris_L, featsHarris_R, mHarris_SAD, opt );
	T = tictac.Tac();
	cout << "[SAD] Matches found: " << mHarris_SAD.size() << " in " << T*1000.0f << " ms " << endl;
	cout << "***************************************************" << endl;
	wind.showImagesAndMatchedPoints( imL, imR, mHarris_SAD, TColor(0,0,255) );

	// SIFT
	cout << "Matching SIFT features by DESCRIPTOR" << endl;
	opt.matching_method = TMatchingOptions::mmDescriptorSIFT;
	//nMatches =
	matchFeatures( featsSIFT_L, featsSIFT_R, mSIFT, opt );
	cout << "Matches found: " << mSIFT.size() << endl;
	cout << "***************************************************" << endl;

	// SURF
	cout << "Matching SURF features by DESCRIPTOR" << endl;
	opt.matching_method = TMatchingOptions::mmDescriptorSURF;
	//nMatches =
	matchFeatures( featsSURF_L, featsSURF_R, mSURF, opt );
	cout << "Matches found: " << mSURF.size() << endl;
	cout << "***************************************************" << endl;

	// FAST
	cout << "Matching FAST features by CC" << endl;
	tictac.Tic();
	//nMatches =
	matchFeatures( featsFAST_L, featsFAST_R, mFAST_CC );
	T = tictac.Tac();
	cout << "[CC] Matches found: " << mFAST_CC.size() << " in " << T*1000.0f << " ms " << endl;

	opt.matching_method = TMatchingOptions::mmSAD;
	tictac.Tic();
	//nMatches =
	matchFeatures( featsFAST_L, featsFAST_R, mFAST_SAD, opt );
	T = tictac.Tac();
	cout << "[SAD] Matches found: " << mFAST_SAD.size() << " in " << T*1000.0f << " ms " << endl;
	cout << "***************************************************" << endl;

	wind2.showImagesAndMatchedPoints( imL, imR, mFAST_SAD, TColor(0,255,0) );

	mrpt::system::pause();

} // end TestMatchFeatures

void TestMatchingComparative()
{
    // Take two images
    string imgL = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imL_p01.jpg");		// Left image
	string imgR = MRPT_EXAMPLES_BASE_DIRECTORY + string("feature_extraction/") + string("imgs/imR_p01.jpg");		// Right image

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

// ------------------------------------------------------
//				TestExtractFeatures
// ------------------------------------------------------
void TestExtractFeatures()
{
	CDisplayWindow		wind1,wind2,wind3,wind4,wind5;
	CFeatureExtraction	fExt;
	CFeatureList		featsHarris, featsKLT, featsSIFT_Hess, featsSIFT_Lowe, featsSIFT_Vedaldi, featsSURF, featsFAST;
	CImage				img;

	if (!img.loadFromFile(the_img_for_extract_feats ))
	{
		cerr << "Cannot load " << the_img_for_extract_feats  << endl;
		return;
	}
	cout << "Loaded test image: " << endl << the_img_for_extract_feats << endl;
	cout << "--------------------------------------------------------------------------" << endl << endl;

	CTicTac	tictac;

	fExt.options.patchSize = 0;

	cout << "Detect Harris features... [f_harris.txt]" << endl;
	tictac.Tic();
	fExt.options.featsType = featHarris;
	fExt.detectFeatures( img, featsHarris );
	cout << "Detected " << featsHarris.size() << " features in ";
	cout << format("  %.03fms",tictac.Tac()*1000) << endl << endl;
	featsHarris.saveToTextFile("f_harris.txt");
	wind1.setWindowTitle("Harris detected features");
	wind1.showImageAndPoints(img, featsHarris);

	cout << "Detect FAST features... [f_fast.txt]" << endl;
	tictac.Tic();
	fExt.options.featsType = featFAST;
	fExt.options.FASTOptions.threshold = 15; //150;
	fExt.options.FASTOptions.min_distance = 4;
	fExt.options.FASTOptions.use_KLT_response = true;
	fExt.detectFeatures( img, featsFAST, 0,  500 /* max num feats */  );
	cout << "Detected " << featsFAST.size() << " features in ";
	cout << format("  %.03fms",tictac.Tac()*1000) << endl << endl;
	featsFAST.saveToTextFile("f_fast.txt");
	wind5.setWindowTitle("FAST detected features");
	wind5.showImageAndPoints( img, featsFAST );

	cout << "Computing SIFT descriptors only ... [f_harris+sift.txt]" << endl;
	tictac.Tic();
	fExt.options.SIFTOptions.implementation = CFeatureExtraction::Hess;
	fExt.computeDescriptors( img, featsHarris, descSIFT );
	cout << format("  %.03fms",tictac.Tac()*1000) << endl << endl;
	featsHarris.saveToTextFile("f_harris+sift.txt");

	cout << "Extracting KLT features... [f_klt.txt]" << endl;
	tictac.Tic();
	fExt.options.featsType = featKLT;
	fExt.options.KLTOptions.threshold	= 0.05f;
	fExt.options.KLTOptions.radius		= 5;
	fExt.detectFeatures( img, featsKLT, 0, 10 );
	cout << "Detected " << featsKLT.size() << " features in ";
	cout << format("  %.03fms",tictac.Tac()*1000) << endl << endl;
	featsKLT.saveToTextFile("f_klt.txt");
	wind2.setWindowTitle("KLT detected features");
	wind2.showImageAndPoints( img, featsKLT );

	cout << "Extracting SIFT features... [f_sift_hess.txt]" << endl;
	tictac.Tic();
	fExt.options.featsType = featSIFT;
	fExt.options.SIFTOptions.implementation = CFeatureExtraction::Hess;
	fExt.detectFeatures( img, featsSIFT_Hess );
	cout << "Detected " << featsSIFT_Hess.size() << " features in ";
	cout << format("  %.03fms",tictac.Tac()*1000) << endl << endl;
	featsSIFT_Hess.saveToTextFile("f_sift_hess.txt");
	wind3.setWindowTitle("SIFT Hess detected features");
	wind3.showImageAndPoints( img, featsSIFT_Hess );

	cout << "Extracting SURF features... [f_surf.txt]" << endl;
	tictac.Tic();
	fExt.options.featsType = featSURF;
	fExt.detectFeatures( img, featsSURF );
	cout << "Detected " << featsSURF.size() << " features in ";
	cout << format("  %.03fms",tictac.Tac()*1000) << endl << endl;
	featsSURF.saveToTextFile("f_surf.txt");
	wind4.setWindowTitle("SURF detected features");
	wind4.showImageAndPoints( img, featsSURF );

	cout << "Computing spin images descriptors only ... [f_harris+spinimgs.txt]" << endl;
	tictac.Tic();
	fExt.options.SpinImagesOptions.radius = 13;
	fExt.options.SpinImagesOptions.hist_size_distance  = 10;
	fExt.options.SpinImagesOptions.hist_size_intensity = 10;
	fExt.computeDescriptors( img, featsHarris, descSpinImages );
	cout << format("  %.03fms",tictac.Tac()*1000) << endl << endl;
	featsHarris.saveToTextFile("f_harris+spinimgs.txt");
	
	mrpt::system::pause();

	return;
}

void TestExtractFeaturesTile()
{
	CDisplayWindow		wind1,wind2;
	CFeatureExtraction	fExt;
	CFeatureList		featsHarris;
	CImage				img;

	string the_img = myDataDir+string("test_image.jpg");

	if (!img.loadFromFile(the_img ))
	{
		cerr << "Cannot load " << the_img  << endl;
		return;
	}
	cout << "Loaded test image: " << the_img << endl;

	CTicTac	tictac;

	cout << "Extracting Harris features (tiled)... [f_harris_tiled.txt]";

	fExt.options.featsType = featHarris;
	fExt.options.harrisOptions.tile_image = true;

	tictac.Tic();
	fExt.detectFeatures( img, featsHarris );
	cout << format("  %.03fms",tictac.Tac()*1000) << endl;

	cout << "Detected " << featsHarris.size() << " features in " << endl;
	featsHarris.saveToTextFile("f_harris_tiled.txt");
	wind1.setWindowTitle("Harris detected features (Tiled image)");
	wind1.showTiledImageAndPoints( img, featsHarris );

	cout << "Extracting Harris features... [f_harris.txt]";

	fExt.options.harrisOptions.tile_image = false;

	tictac.Tic();
	fExt.detectFeatures( img, featsHarris );
	cout << format("  %.03fms",tictac.Tac()*1000) << endl;

	featsHarris.saveToTextFile("f_harris.txt");
	wind2.setWindowTitle("Harris detected features");
	wind2.showTiledImageAndPoints( img, featsHarris );

	mrpt::system::pause();

	return;
}

int main(int argc, char **argv)
{
	try
	{
        TestMatchFeatures();
		//TestExtractFeatures();
		//TestExtractFeaturesTile();
		//TestTrackFeatures();
		// TestMatchingComparative();
        //TestORBTiled();


//		CFeatureList  fs;
//		fs.loadFromTextFile("f_harris+sift.txt");
//		fs.saveToTextFile("f_harris+sift2.txt");

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
