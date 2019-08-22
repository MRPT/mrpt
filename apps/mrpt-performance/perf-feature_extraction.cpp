/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
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
//				Benchmark: FAST9/10/12
// ------------------------------------------------------
#define GENERATE_BENCHMARK_FASTERS(__BENCH_FUNC__, __FAST_FUNC_NAME__)      \
	template <int W, int H, bool ROW_INDEX>                                 \
	double __BENCH_FUNC__(int N, int dummy)                                 \
	{                                                                       \
		CImage img;                                                         \
		getTestImage(0, img);                                               \
		img = img.grayscale();                                              \
		img.scaleImage(img, W, H, mrpt::img::IMG_INTERP_LINEAR);            \
		TKeyPointList corners;                                              \
		const int threshold = 20;                                           \
		std::vector<size_t> feats_index_by_row;                             \
		CTicTac tictac;                                                     \
		tictac.Tic();                                                       \
		for (int i = 0; i < N; i++)                                         \
		{                                                                   \
			CFeatureExtraction::__FAST_FUNC_NAME__(                         \
				img, corners, threshold, false /*don't append*/,            \
				0 /* octave */, ROW_INDEX ? &feats_index_by_row : nullptr); \
		}                                                                   \
		const double T = tictac.Tac() / N;                                  \
		return T;                                                           \
	}

GENERATE_BENCHMARK_FASTERS(
	benchmark_detectFeatures_FAST9, detectFeatures_SSE2_FASTER9)
GENERATE_BENCHMARK_FASTERS(
	benchmark_detectFeatures_FAST10, detectFeatures_SSE2_FASTER10)
GENERATE_BENCHMARK_FASTERS(
	benchmark_detectFeatures_FAST12, detectFeatures_SSE2_FASTER12)

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
	fExt.options.featsType = featFASTER9;

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
	for (int i = 0; i < N; i++) fExt.detectFeatures(img, feats, 0, MAX_N_FEATS);

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
	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-9 (libCVD)",
		benchmark_detectFeatures<featFASTER9>, 100);
	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-10 (libCVD)",
		benchmark_detectFeatures<featFASTER10>, 100);
	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-12 (libCVD)",
		benchmark_detectFeatures<featFASTER12>, 100);
	MRPT_TODO("AKAZE crashes inside OpenCV. Disabled for now (Jan 2019)");
#if 0
	lstTests.emplace_back(
		"feature_extraction [640x480]: AKAZE (OpenCV)",
		benchmark_detectFeatures<featAKAZE>, 5);
#endif
	lstTests.emplace_back(
		"feature_extraction [640x480]: LSD (OpenCV)",
		benchmark_detectFeatures<featLSD>, 5);

	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-9 (libcvd)",
		benchmark_detectFeatures_FASTER<featFASTER9, 0>, 100, 20);
	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-9 (sorted best 200) (libcvd)",
		benchmark_detectFeatures_FASTER<featFASTER9, 200>, 100, 20);

	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-10  (libcvd)",
		benchmark_detectFeatures_FASTER<featFASTER10, 0>, 100, 20);
	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-10 (sorted best 200) (libcvd)",
		benchmark_detectFeatures_FASTER<featFASTER10, 200>, 100, 20);

	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-12  (libcvd)",
		benchmark_detectFeatures_FASTER<featFASTER12, 0>, 100, 20);
	lstTests.emplace_back(
		"feature_extraction [640x480]: FASTER-12 (sorted best 200) (libcvd)",
		benchmark_detectFeatures_FASTER<featFASTER12, 200>, 100, 20);

	lstTests.emplace_back(
		"feature_extraction [640x480]: detectFeatures_SSE2_FASTER9() (libcvd)",
		benchmark_detectFeatures_FAST9<640, 480, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [640x480]: detectFeatures_SSE2_FASTER10() (libcvd)",
		benchmark_detectFeatures_FAST10<640, 480, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [640x480]: detectFeatures_SSE2_FASTER12() (libcvd)",
		benchmark_detectFeatures_FAST12<640, 480, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [640x480]: "
		"detectFeatures_SSE2_FASTER9()+row-index (libcvd)",
		benchmark_detectFeatures_FAST9<640, 480, true>, 1000);
	lstTests.emplace_back(
		"feature_extraction [640x480]: "
		"detectFeatures_SSE2_FASTER10()+row-index (libcvd)",
		benchmark_detectFeatures_FAST10<640, 480, true>, 1000);
	lstTests.emplace_back(
		"feature_extraction [640x480]: "
		"detectFeatures_SSE2_FASTER12()+row-index (libcvd)",
		benchmark_detectFeatures_FAST12<640, 480, true>, 1000);

	lstTests.emplace_back(
		"feature_extraction [800x600]: detectFeatures_SSE2_FASTER9() (libcvd)",
		benchmark_detectFeatures_FAST9<800, 600, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [800x600]: detectFeatures_SSE2_FASTER10() (libcvd)",
		benchmark_detectFeatures_FAST10<800, 600, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [800x600]: detectFeatures_SSE2_FASTER12() (libcvd)",
		benchmark_detectFeatures_FAST12<800, 600, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [800x600]: "
		"detectFeatures_SSE2_FASTER9()+row-index (libcvd)",
		benchmark_detectFeatures_FAST9<800, 600, true>, 1000);
	lstTests.emplace_back(
		"feature_extraction [800x600]: "
		"detectFeatures_SSE2_FASTER10()+row-index (libcvd)",
		benchmark_detectFeatures_FAST10<800, 600, true>, 1000);
	lstTests.emplace_back(
		"feature_extraction [800x600]: "
		"detectFeatures_SSE2_FASTER12()+row-index (libcvd)",
		benchmark_detectFeatures_FAST12<800, 600, true>, 1000);

	lstTests.emplace_back(
		"feature_extraction [1024x768]: detectFeatures_SSE2_FASTER9() (libcvd)",
		benchmark_detectFeatures_FAST9<1024, 768, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [1024x768]: detectFeatures_SSE2_FASTER10() "
		"(libcvd)",
		benchmark_detectFeatures_FAST10<1024, 768, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [1024x768]: detectFeatures_SSE2_FASTER12() "
		"(libcvd)",
		benchmark_detectFeatures_FAST12<1024, 768, false>, 1000);
	lstTests.emplace_back(
		"feature_extraction [1024x768]: "
		"detectFeatures_SSE2_FASTER9()+row-index (libcvd)",
		benchmark_detectFeatures_FAST9<1024, 768, true>, 1000);
	lstTests.emplace_back(
		"feature_extraction [1024x768]: "
		"detectFeatures_SSE2_FASTER10()+row-index (libcvd)",
		benchmark_detectFeatures_FAST10<1024, 768, true>, 1000);
	lstTests.emplace_back(
		"feature_extraction [1024x768]: "
		"detectFeatures_SSE2_FASTER12()+row-index (libcvd)",
		benchmark_detectFeatures_FAST12<1024, 768, true>, 1000);

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
