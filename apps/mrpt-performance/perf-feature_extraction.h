/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
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

using namespace mrpt::vision;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::gui;
using namespace mrpt::system;
using namespace mrpt::scanmatching;
using namespace std;

extern void getTestImage(unsigned int img_index, mrpt::utils::CImage &out_img );

// ------------------------------------------------------
//				Benchmark: Harris
// ------------------------------------------------------
double feature_extraction_test_Harris( int w, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img(w,h,3);
	//for (int i=0;i<5000;i++)
	//	img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), randomGenerator.drawUniform32bit() );

	getTestImage(0,img);	// img.loadFromFile("c:/temp/img_0000.jpg");

	CFeatureExtraction	fExt;
	CFeatureList		featsHarris;

	const size_t		N = 10;
	fExt.options.featsType = featHarris;
	tictac.Tic();
	for (size_t i=0;i<N;i++)
		fExt.detectFeatures( img, featsHarris );

	const double T = tictac.Tac()/N;

//	cout << "Harris: " << featsHarris.size();

	return T;
}

// ------------------------------------------------------
//				Benchmark: KLT
// ------------------------------------------------------
double feature_extraction_test_KLT( int w, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img(w,h,3);
	//for (int i=0;i<5000;i++)
	//	img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), randomGenerator.drawUniform32bit() );

	getTestImage(0,img);	// img.loadFromFile("c:/temp/img_0000.jpg");

	CFeatureExtraction	fExt;
	CFeatureList		featsKLT;
	const size_t		N = 10;

	fExt.options.featsType				= featKLT;
	fExt.options.KLTOptions.threshold	= 0.05f;
	fExt.options.KLTOptions.radius		= 5;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		fExt.detectFeatures( img, featsKLT );

	const double T = tictac.Tac()/N;
//	cout << "SPIN: " << featsKLT.size();
	return T;
}

// ------------------------------------------------------
//				Benchmark: SIFT (hess)
// ------------------------------------------------------
double feature_extraction_test_SIFT( int w, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img(w,h,3);
	//for (int i=0;i<5000;i++)
	//	img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), randomGenerator.drawUniform32bit() );

	getTestImage(0,img);	// img.loadFromFile("c:/temp/img_0000.jpg");

	CFeatureExtraction	fExt;
	CFeatureList		featsSIFT;
	const size_t		N = 5;

	fExt.options.featsType	= featSIFT;
	fExt.options.SIFTOptions.implementation = CFeatureExtraction::Hess;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		fExt.detectFeatures( img, featsSIFT );

	const double T = tictac.Tac()/N;

//	cout << "SIFT: " << featsSIFT.size();

	return T;
}

// ------------------------------------------------------
//				Benchmark: SIFT descriptor only
// ------------------------------------------------------
double feature_extraction_test_SIFT_desc( int w, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img(w,h,3);
	/*for (int i=0;i<5000;i++)
		img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), randomGenerator.drawUniform32bit() );*/

	getTestImage(0,img);	// img.loadFromFile("c:/temp/img_0000.jpg");

	CFeatureExtraction	fExt;
	CFeatureList		featsHarris;
	const size_t		N = 5;

	fExt.options.featsType	= featHarris;

	fExt.detectFeatures( img, featsHarris );

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		fExt.computeDescriptors( img, featsHarris, descSIFT );

	const double T = tictac.Tac()/N;

//	cout << "SIFT desc: " << featsHarris.size();
	return T;
}

// ------------------------------------------------------
//				Benchmark: SURF
// ------------------------------------------------------
double feature_extraction_test_SURF( int w, int h )
{

	CTicTac	 tictac;

	// Generate a random image
	CImage  img(w,h,3);
	//for (int i=0;i<5000;i++)
	//	img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), randomGenerator.drawUniform32bit() );
	getTestImage(0,img);	// img.loadFromFile("c:/temp/img_0000.jpg");

	CFeatureExtraction	fExt;
	CFeatureList		featsSURF;
	const size_t		N = 10;

	fExt.options.featsType					= featSURF;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		fExt.detectFeatures( img, featsSURF );

	const double T = tictac.Tac()/N;

//	cout << "SURF: " << featsSURF.size();

	return T;
}

// ------------------------------------------------------
//				Benchmark: FAST
// ------------------------------------------------------
double feature_extraction_test_FAST( int w, int h )
{
	CTicTac			tictac;

	// Generate a random image
	CImage  img(w,h,3);

	getTestImage(0,img);	// img.loadFromFile("c:/temp/img_0000.jpg");

	CFeatureExtraction		fExt;
	CFeatureList			featsFAST;
	const size_t			N = 100;

	fExt.options.featsType	= featFAST;
	fExt.options.FASTOptions.threshold = 20;
	fExt.options.patchSize = 0;

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		fExt.detectFeatures( img, featsFAST );

	const double T = tictac.Tac()/N;

//	cout << "FAST: " << featsFAST.size() << endl;
	return T;
}

// ------------------------------------------------------
//				Benchmark: Spin descriptor
// ------------------------------------------------------
double feature_extraction_test_Spin_desc( int w, int h )
{
	CTicTac	 tictac;

	// Generate a random image
	CImage  img(w,h,3);
	//for (int i=0;i<5000;i++)
	//	img.line(randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1),randomGenerator.drawUniform(0,w-1),randomGenerator.drawUniform(0,h-1), randomGenerator.drawUniform32bit() );

	getTestImage(0,img);	// img.loadFromFile("c:/temp/img_0000.jpg");
	CFeatureExtraction	fExt;
	CFeatureList		featsHarris;
	const size_t		N = 20;

	fExt.options.SpinImagesOptions.radius				= 13;
	fExt.options.SpinImagesOptions.hist_size_distance	= 10;
	fExt.options.SpinImagesOptions.hist_size_intensity	= 10;

	fExt.detectFeatures( img, featsHarris );

	tictac.Tic();
	for (size_t i=0;i<N;i++)
		fExt.computeDescriptors( img, featsHarris, descSpinImages );

	const double T = tictac.Tac()/N;

//	cout << "SPIN desc: " << featsHarris.size();

	return T;
}

// ------------------------------------------------------
// register_tests_feature_extraction
// ------------------------------------------------------
void register_tests_feature_extraction()
{
	lstTests.push_back( TestData("feature_extraction [640x480]: Harris", feature_extraction_test_Harris, 640, 480 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: KLT", feature_extraction_test_KLT, 640, 480 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: SIFT", feature_extraction_test_SIFT, 640, 480 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: SIFT desc.", feature_extraction_test_SIFT_desc, 640, 480 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: SURF", feature_extraction_test_SURF, 640, 480 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: FAST", feature_extraction_test_FAST, 640, 480 ) );
	lstTests.push_back( TestData("feature_extraction [640x480]: Spin desc.", feature_extraction_test_Spin_desc, 640, 480 ) );
}
