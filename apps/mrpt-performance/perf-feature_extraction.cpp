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

extern void getTestImage(unsigned int img_index, mrpt::utils::CImage &out_img );

// ------------------------------------------------------
//				Benchmark: Harris
// ------------------------------------------------------
double feature_extraction_test_Harris( int N, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img;
	getTestImage(0,img);

	CFeatureExtraction	fExt;
	CFeatureList		featsHarris;

	fExt.options.featsType = featHarris;
	tictac.Tic();
	for (int i=0;i<N;i++)
		fExt.detectFeatures( img, featsHarris );

	const double T = tictac.Tac()/N;

//	cout << "Harris: " << featsHarris.size();

	return T;
}

// ------------------------------------------------------
//				Benchmark: KLT
// ------------------------------------------------------
double feature_extraction_test_KLT( int N, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img;
	getTestImage(0,img);

	CFeatureExtraction	fExt;
	CFeatureList		featsKLT;

	fExt.options.featsType				= featKLT;
	fExt.options.KLTOptions.threshold	= 0.05f;
	fExt.options.KLTOptions.radius		= 5;

	tictac.Tic();
	for (int i=0;i<N;i++)
		fExt.detectFeatures( img, featsKLT );

	const double T = tictac.Tac()/N;
//	cout << "SPIN: " << featsKLT.size();
	return T;
}

// ------------------------------------------------------
//				Benchmark: SIFT (hess)
// ------------------------------------------------------
double feature_extraction_test_SIFT( int N, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img;
	getTestImage(0,img);

	CFeatureExtraction	fExt;
	CFeatureList		featsSIFT;

	fExt.options.featsType	= featSIFT;
	fExt.options.SIFTOptions.implementation = CFeatureExtraction::Hess;

	tictac.Tic();
	for (int i=0;i<N;i++)
		fExt.detectFeatures( img, featsSIFT );

	const double T = tictac.Tac()/N;

//	cout << "SIFT: " << featsSIFT.size();

	return T;
}

// ------------------------------------------------------
//				Benchmark: SIFT descriptor only
// ------------------------------------------------------
double feature_extraction_test_SIFT_desc( int N, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img;
	getTestImage(0,img);

	CFeatureExtraction	fExt;
	CFeatureList		featsHarris;

	fExt.options.featsType	= featHarris;

	fExt.detectFeatures( img, featsHarris );

	tictac.Tic();
	for (int i=0;i<N;i++)
		fExt.computeDescriptors( img, featsHarris, descSIFT );

	const double T = tictac.Tac()/N;

//	cout << "SIFT desc: " << featsHarris.size();
	return T;
}

// ------------------------------------------------------
//				Benchmark: SURF
// ------------------------------------------------------
double feature_extraction_test_SURF( int N, int h )
{

	CTicTac	 tictac;

	// Generate a random image
	CImage  img;
	getTestImage(0,img);

	CFeatureExtraction	fExt;
	CFeatureList		featsSURF;

	fExt.options.featsType					= featSURF;

	tictac.Tic();
	for (int i=0;i<N;i++)
		fExt.detectFeatures( img, featsSURF );

	const double T = tictac.Tac()/N;

//	cout << "SURF: " << featsSURF.size();

	return T;
}

// ------------------------------------------------------
//				Benchmark: FAST
// ------------------------------------------------------
double feature_extraction_test_FAST( int N, int h )
{
	CTicTac			tictac;

	// Generate a random image
	CImage  img;
	getTestImage(0,img);

	CFeatureExtraction		fExt;
	CFeatureList			featsFAST;

	fExt.options.featsType	= featFAST;
	fExt.options.FASTOptions.threshold = 20;
	fExt.options.patchSize = 0;

	img.grayscaleInPlace();

	tictac.Tic();
	for (int i=0;i<N;i++)
		fExt.detectFeatures( img, featsFAST );

	const double T = tictac.Tac()/N;
	return T;
}

// ------------------------------------------------------
//				Benchmark: FAST9/10/12
// ------------------------------------------------------
#define GENERATE_BENCHMARK_FASTERS(__BENCH_FUNC__, __FAST_FUNC_NAME__) \
	template <int W,int H, bool ROW_INDEX> \
	double __BENCH_FUNC__( int N, int dummy ) \
	{ \
		CImage img; \
		getTestImage(0,img); \
		img.grayscaleInPlace(); \
		img.scaleImage(W,H); \
		TSimpleFeatureList corners; \
		const int threshold = 20; \
		std::vector<size_t> feats_index_by_row; \
		CTicTac tictac; \
		tictac.Tic(); \
		for (int i=0;i<N;i++) \
		{ \
			CFeatureExtraction::__FAST_FUNC_NAME__( \
				img, \
				corners,  \
				threshold,  \
				false /*don't append*/, \
				0 /* octave */, \
				ROW_INDEX ? &feats_index_by_row : NULL \
				); \
		} \
		const double T = tictac.Tac()/N; \
		return T; \
	}

GENERATE_BENCHMARK_FASTERS(feature_extraction_test_FAST9,  detectFeatures_SSE2_FASTER9)
GENERATE_BENCHMARK_FASTERS(feature_extraction_test_FAST10, detectFeatures_SSE2_FASTER10)
GENERATE_BENCHMARK_FASTERS(feature_extraction_test_FAST12, detectFeatures_SSE2_FASTER12)

// ------------------------------------------------------
//				Benchmark: Spin descriptor
// ------------------------------------------------------
double feature_extraction_test_Spin_desc( int N, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img;
	getTestImage(0,img);

	CFeatureExtraction	fExt;
	CFeatureList		featsHarris;

	fExt.options.SpinImagesOptions.radius				= 13;
	fExt.options.SpinImagesOptions.hist_size_distance	= 10;
	fExt.options.SpinImagesOptions.hist_size_intensity	= 10;

	fExt.detectFeatures( img, featsHarris );

	tictac.Tic();
	for (int i=0;i<N;i++)
		fExt.computeDescriptors( img, featsHarris, descSpinImages );

	const double T = tictac.Tac()/N;
	return T;
}

// ------------------------------------------------------
//				Benchmark: FASTER
// ------------------------------------------------------
template <mrpt::vision::TFeatureType TYP, int MAX_N_FEATS>
double feature_extraction_test_FASTER( int N, int threshold )
{
	CTicTac			tictac;

	// Generate a random image
	CImage  img;
	getTestImage(0,img);

	CFeatureExtraction		fExt;
	CFeatureList			feats;

	fExt.options.featsType	= TYP; // FASTER_N==9 ? featFASTER9 : (FASTER_N==10 ? featFASTER10 : featFASTER12 );
	fExt.options.FASTOptions.threshold = threshold; //20;
	fExt.options.patchSize = 0;

	img.grayscaleInPlace();

	tictac.Tic();
	for (int i=0;i<N;i++)
		fExt.detectFeatures( img, feats,0, MAX_N_FEATS );

	const double T = tictac.Tac()/N;
	return T;
}


// ------------------------------------------------------
// register_tests_feature_extraction
// ------------------------------------------------------
void register_tests_feature_extraction()
{
	lstTests.push_back( TestData("feature_extraction [640x480]: Harris", feature_extraction_test_Harris, 30  ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: KLT", feature_extraction_test_KLT, 30  ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: SIFT", feature_extraction_test_SIFT, 5  ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: SIFT desc.", feature_extraction_test_SIFT_desc, 5  ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: SURF", feature_extraction_test_SURF, 10  ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: FAST", feature_extraction_test_FAST, 100  ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: Spin desc.", feature_extraction_test_Spin_desc, 30  ) );

	lstTests.push_back( TestData("feature_extraction [640x480]: FASTER-9", feature_extraction_test_FASTER<featFASTER9,0>, 100 , 20 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: FASTER-9 (sorted best 200)", feature_extraction_test_FASTER<featFASTER9,200>, 100 , 20 ) );

	lstTests.push_back( TestData("feature_extraction [640x480]: FASTER-10", feature_extraction_test_FASTER<featFASTER10,0>, 100 , 20 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: FASTER-10 (sorted best 200)", feature_extraction_test_FASTER<featFASTER10,200>, 100 , 20) );

	lstTests.push_back( TestData("feature_extraction [640x480]: FASTER-12", feature_extraction_test_FASTER<featFASTER12,0>, 100 , 20 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: FASTER-12 (sorted best 200)", feature_extraction_test_FASTER<featFASTER12,200>, 100 , 20 ) );

	lstTests.push_back( TestData("feature_extraction [640x480]: detectFeatures_SSE2_FASTER9()", feature_extraction_test_FAST9<640,480,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: detectFeatures_SSE2_FASTER10()", feature_extraction_test_FAST10<640,480,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: detectFeatures_SSE2_FASTER12()", feature_extraction_test_FAST12<640,480,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: detectFeatures_SSE2_FASTER9()+row-index", feature_extraction_test_FAST9<640,480,true>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: detectFeatures_SSE2_FASTER10()+row-index", feature_extraction_test_FAST10<640,480,true>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: detectFeatures_SSE2_FASTER12()+row-index", feature_extraction_test_FAST12<640,480,true>, 1000 ) );

	lstTests.push_back( TestData("feature_extraction [800x600]: detectFeatures_SSE2_FASTER9()", feature_extraction_test_FAST9<800,600,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [800x600]: detectFeatures_SSE2_FASTER10()", feature_extraction_test_FAST10<800,600,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [800x600]: detectFeatures_SSE2_FASTER12()", feature_extraction_test_FAST12<800,600,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [800x600]: detectFeatures_SSE2_FASTER9()+row-index", feature_extraction_test_FAST9<800,600,true>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [800x600]: detectFeatures_SSE2_FASTER10()+row-index", feature_extraction_test_FAST10<800,600,true>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [800x600]: detectFeatures_SSE2_FASTER12()+row-index", feature_extraction_test_FAST12<800,600,true>, 1000 ) );

	lstTests.push_back( TestData("feature_extraction [1024x768]: detectFeatures_SSE2_FASTER9()", feature_extraction_test_FAST9<1024,768,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [1024x768]: detectFeatures_SSE2_FASTER10()", feature_extraction_test_FAST10<1024,768,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [1024x768]: detectFeatures_SSE2_FASTER12()", feature_extraction_test_FAST12<1024,768,false>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [1024x768]: detectFeatures_SSE2_FASTER9()+row-index", feature_extraction_test_FAST9<1024,768,true>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [1024x768]: detectFeatures_SSE2_FASTER10()+row-index", feature_extraction_test_FAST10<1024,768,true>, 1000 ) );
	lstTests.push_back( TestData("feature_extraction [1024x768]: detectFeatures_SSE2_FASTER12()+row-index", feature_extraction_test_FAST12<1024,768,true>, 1000 ) );

}
