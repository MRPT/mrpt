/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/utils/CImage.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include "common.h"

using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

const unsigned int NFEATS = 100;

extern void getTestImage(unsigned int img_index, mrpt::utils::CImage &out_img );

// ------------------------------------------------------
//				Benchmark: Harris + CC
// ------------------------------------------------------
double feature_matching_test_Harris_CC( int w, int h )
{
	CTicTac	 tictac;

	CImage  imL, imR;
	CFeatureExtraction	fExt;
	CFeatureList		featsHarris_L, featsHarris_R;
	CMatchedFeatureList	mHarris;

	getTestImage(0,imR);
	getTestImage(1,imL);

	// Extract features: HARRIS
	fExt.options.featsType = featHarris;

	//size_t				nMatches;
	TMatchingOptions	opt;
	const size_t		N = 20;

	// HARRIS
	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
		fExt.detectFeatures( imL, featsHarris_L, 0, NFEATS );
		fExt.detectFeatures( imR, featsHarris_R, 0, NFEATS );
		//nMatches =
		matchFeatures( featsHarris_L, featsHarris_R, mHarris );
	}
	const double T = tictac.Tac()/N;

//	cout << endl << "L: " << featsHarris_L.size() << " R: " << featsHarris_R.size() << " M: " << mHarris.size() << endl;


	return T;
}

// ------------------------------------------------------
//				Benchmark: Harris + SAD
// ------------------------------------------------------
double feature_matching_test_Harris_SAD( int w, int h )
{
	CTicTac	 tictac;

	CImage  imL, imR;
	CFeatureExtraction	fExt;
	CFeatureList		featsHarris_L, featsHarris_R;
	CMatchedFeatureList	mHarris;

	getTestImage(0,imR);
	getTestImage(1,imL);

	// Extract features: HARRIS
	fExt.options.featsType = featHarris;

	TMatchingOptions	opt;
	const size_t		N = 20;
	opt.matching_method = TMatchingOptions::mmSAD;

	// HARRIS
	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
		fExt.detectFeatures( imL, featsHarris_L, 0, NFEATS );
		fExt.detectFeatures( imR, featsHarris_R, 0, NFEATS );
		//nMatches =
		matchFeatures( featsHarris_L, featsHarris_R, mHarris, opt );
	}
	const double T = tictac.Tac()/N;

//	cout << endl << "L: " << featsHarris_L.size() << " R: " << featsHarris_R.size() << " M: " << mHarris.size() << endl;

	return T;
}

// ------------------------------------------------------
//				Benchmark: Harris + SAD
// ------------------------------------------------------
double feature_matching_test_SIFT( int w, int h )
{
	CTicTac	 tictac;

	CImage  imL, imR;
	CFeatureExtraction	fExt;
	CFeatureList		featsSIFT_L, featsSIFT_R;
	CMatchedFeatureList	mSIFT;

	getTestImage(0,imR);
	getTestImage(1,imL);

	// Extract features: HARRIS
	fExt.options.featsType = featSIFT;

	//size_t				nMatches;
	TMatchingOptions	opt;
	const size_t		N = 5;
	opt.matching_method = TMatchingOptions::mmDescriptorSIFT;

	// HARRIS
	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
		fExt.detectFeatures( imL, featsSIFT_L, 0, NFEATS );
		fExt.detectFeatures( imR, featsSIFT_R, 0, NFEATS );
		//nMatches =
		matchFeatures( featsSIFT_L, featsSIFT_R, mSIFT, opt );
	}
	const double T = tictac.Tac()/N;

//	cout << endl << "L: " << featsSIFT_L.size() << " R: " << featsSIFT_R.size() << " M: " << mSIFT.size() << endl;

	return T;
}

// ------------------------------------------------------
//				Benchmark: Harris + SAD
// ------------------------------------------------------
double feature_matching_test_SURF( int w, int h )
{
	CTicTac	 tictac;

	CImage  imL, imR;
	CFeatureExtraction	fExt;
	CFeatureList		featsSURF_L, featsSURF_R;
	CMatchedFeatureList	mSURF;

	getTestImage(0,imR);
	getTestImage(1,imL);

	// Extract features: HARRIS
	fExt.options.featsType = featSURF;

	//size_t				nMatches;
	TMatchingOptions	opt;
	const size_t		N = 10;
	opt.matching_method = TMatchingOptions::mmDescriptorSURF;

	// HARRIS
	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
		fExt.detectFeatures( imL, featsSURF_L, 0, NFEATS );
		fExt.detectFeatures( imR, featsSURF_R, 0, NFEATS );
		//nMatches =
		matchFeatures( featsSURF_L, featsSURF_R, mSURF, opt );
	}
	const double T = tictac.Tac()/N;

//	cout << endl << "L: " << featsSURF_L.size() << " R: " << featsSURF_R.size() << " M: " << mSURF.size() << endl;

	return T;
}

// ------------------------------------------------------
//				Benchmark: Harris + SAD
// ------------------------------------------------------
double feature_matching_test_FAST_CC( int w, int h )
{
	CTicTac	 tictac;

	CImage  imL, imR;
	CFeatureExtraction	fExt;
	CFeatureList		featsFAST_L, featsFAST_R;
	CMatchedFeatureList	mFAST;

	getTestImage(0,imR);
	getTestImage(1,imL);

	// Extract features: HARRIS
	fExt.options.featsType = featFAST;

	//size_t				nMatches;
	TMatchingOptions	opt;
	const size_t		N = 20;

	// HARRIS
	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
		fExt.detectFeatures( imL, featsFAST_L, 0, NFEATS );
		fExt.detectFeatures( imR, featsFAST_R, 0, NFEATS );
		//nMatches =
		matchFeatures( featsFAST_L, featsFAST_R, mFAST, opt );
	}
	const double T = tictac.Tac()/N;

//	cout << endl << "L: " << featsFAST_L.size() << " R: " << featsFAST_R.size() << " M: " << mFAST.size() << endl;

	return T;
}

// ------------------------------------------------------
//				Benchmark: Harris + SAD
// ------------------------------------------------------
double feature_matching_test_FAST_SAD( int w, int h )
{
	CTicTac	 tictac;

	CImage  imL, imR;
	CFeatureExtraction	fExt;
	CFeatureList		featsFAST_L, featsFAST_R;
	CMatchedFeatureList	mFAST;

	getTestImage(0,imR);
	getTestImage(1,imL);

	// Extract features: HARRIS
	fExt.options.featsType = featFAST;

	//size_t				nMatches;
	TMatchingOptions	opt;
	const size_t		N = 20;
	opt.matching_method			= TMatchingOptions::mmSAD;

	// HARRIS
	tictac.Tic();
	for (size_t i=0;i<N;i++)
	{
		fExt.detectFeatures( imL, featsFAST_L, 0, NFEATS );
		fExt.detectFeatures( imR, featsFAST_R, 0, NFEATS );
		//nMatches =
		matchFeatures( featsFAST_L, featsFAST_R, mFAST, opt );
	}
	const double T = tictac.Tac()/N;

//	cout << endl << "L: " << featsFAST_L.size() << " R: " << featsFAST_R.size() << " M: " << mFAST.size() << endl;

	return T;
}

// ------------------------------------------------------
// register_tests_feature_extraction
// ------------------------------------------------------
void register_tests_feature_matching()
{
	lstTests.push_back( TestData("feature_matching [640x480]: Harris + CC", feature_matching_test_Harris_CC, 640, 480 ) );
	lstTests.push_back( TestData("feature_matching [640x480]: Harris + SAD", feature_matching_test_Harris_SAD, 640, 480 ) );
	lstTests.push_back( TestData("feature_matching [640x480]: SIFT", feature_matching_test_SIFT, 640, 480 ) );
	lstTests.push_back( TestData("feature_matching [640x480]: SURF", feature_matching_test_SURF, 640, 480 ) );
	lstTests.push_back( TestData("feature_matching [640x480]: FAST + CC", feature_matching_test_FAST_CC, 640, 480 ) );
	lstTests.push_back( TestData("feature_matching [640x480]: FAST + SAD", feature_matching_test_FAST_SAD, 640, 480 ) );
}
