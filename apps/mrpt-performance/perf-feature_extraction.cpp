/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include <mrpt/img/CImage.h>
#include <mrpt/vision/CFeatureExtraction.h>

#include <iomanip>

#include "common.h"

using namespace mrpt::vision;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

extern void getTestImage(unsigned int img_index, mrpt::img::CImage& out_img);

template <TKeyPointMethod FEAT_TYPE>
double benchmark_detectFeatures(int N, [[maybe_unused]] int h)
{
	// Generate a random image
	CImage img;
	getTestImage(0, img);
	CFeatureExtraction fExt;
	fExt.profiler.enable();
	fExt.options.featsType = FEAT_TYPE;
	for (int i = 0; i < N; i++)
	{
		CFeatureList fs;
		fExt.detectFeatures(img, fs);
		if (i == (N - 1))
			std::cout << "(" << std::setw(4) << fs.size() << " found)\n";
	}
	return fExt.profiler.getMeanTime("detectFeatures");
}

// ------------------------------------------------------
//				Benchmark: descriptor
// ------------------------------------------------------
template <TDescriptorType DESCRIPTOR_TYPE>
double benchmark_computeDescriptor(int N, int num_feats)
{
	CImage img;
	getTestImage(0, img);

	CFeatureExtraction fExt;
	fExt.profiler.enable();
	fExt.options.featsType = featFAST;

	for (int i = 0; i < N; i++)
	{
		CFeatureList fs;
		fExt.detectFeatures(img, fs, 0 /*id*/, num_feats);
		fExt.computeDescriptors(img, fs, DESCRIPTOR_TYPE);
	}

	return fExt.profiler.getMeanTime("computeDescriptors");
}

// ------------------------------------------------------
//				Benchmark: FASTER
// ------------------------------------------------------
template <mrpt::vision::TKeyPointMethod TYP, int MAX_N_FEATS>
double benchmark_detectFeatures_FASTER(int N, int threshold)
{
	CTicTac tictac;

	// Generate a random image
	CImage img;
	getTestImage(0, img);

	CFeatureExtraction fExt;
	CFeatureList feats;

	fExt.options.featsType = TYP;
	fExt.options.FASTOptions.threshold = threshold;
	fExt.options.patchSize = 0;

	img = img.grayscale();

	tictac.Tic();
	for (int i = 0; i < N; i++)
		fExt.detectFeatures(img, feats, 0, MAX_N_FEATS);

	const double T = tictac.Tac() / N;
	return T;
}

// ------------------------------------------------------
// register_tests_feature_extraction
// ------------------------------------------------------
void register_tests_feature_extraction()
{
	// Detectors:
	lstTests.emplace_back(
		"feature_extraction [640x480]: Harris (OpenCV)",
		benchmark_detectFeatures<featHarris>, 30);
	lstTests.emplace_back(
		"feature_extraction [640x480]: KLT (OpenCV)",
		benchmark_detectFeatures<featKLT>, 30);
	lstTests.emplace_back(
		"feature_extraction [640x480]: SIFT detect (OpenCV)",
		benchmark_detectFeatures<featSIFT>, 5);
	lstTests.emplace_back(
		"feature_extraction [640x480]: SURF (OpenCV)",
		benchmark_detectFeatures<featSURF>, 10);
	lstTests.emplace_back(
		"feature_extraction [640x480]: ORB (OpenCV)",
		benchmark_detectFeatures<featORB>, 10);
	lstTests.emplace_back(
		"feature_extraction [640x480]: FAST (OpenCV)",
		benchmark_detectFeatures<featFAST>, 100);

	MRPT_TODO("AKAZE crashes inside OpenCV. Disabled for now (Jan 2019)");
#if 0
	lstTests.emplace_back(
		"feature_extraction [640x480]: AKAZE (OpenCV)",
		benchmark_detectFeatures<featAKAZE>, 5);
#endif
	lstTests.emplace_back(
		"feature_extraction [640x480]: LSD (OpenCV)",
		benchmark_detectFeatures<featLSD>, 5);

	// Descriptors:
	lstTests.emplace_back(
		"feature_computeDescriptor [640x480,N=100]: ORB (OpenCV)",
		benchmark_computeDescriptor<descORB>, 30, 100);
	lstTests.emplace_back(
		"feature_computeDescriptor [640x480,N=100]: Spin (OpenCV)",
		benchmark_computeDescriptor<descSpinImages>, 30, 100);
	lstTests.emplace_back(
		"feature_computeDescriptor [640x480,N=100]: SURF (OpenCV)",
		benchmark_computeDescriptor<descSURF>, 6, 100);
	/*	lstTests.emplace_back(
			"feature_computeDescriptor [640x480,N=100]: SIFT (OpenCV)",
			benchmark_computeDescriptor<descSIFT>, 6, 100);
			*/
}
