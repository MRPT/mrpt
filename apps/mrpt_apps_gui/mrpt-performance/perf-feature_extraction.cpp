/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

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
  // Note: CFeatureExtraction is not yet implemented in v3.0
  // This is a stub that returns 0
  return 0;
}

// ------------------------------------------------------
//				Benchmark: descriptor
// ------------------------------------------------------
template <TDescriptorType DESCRIPTOR_TYPE>
double benchmark_computeDescriptor(int N, int num_feats)
{
  // Note: CFeatureExtraction is not yet implemented in v3.0
  // This is a stub that returns 0
  return 0;
}

// ------------------------------------------------------
//				Benchmark: FASTER
// ------------------------------------------------------
template <mrpt::vision::TKeyPointMethod TYP, int MAX_N_FEATS>
double benchmark_detectFeatures_FASTER(int N, int threshold)
{
  // Note: CFeatureExtraction is not yet implemented in v3.0
  // This is a stub that returns 0
  return 0;
}

// ------------------------------------------------------
// register_tests_feature_extraction
// ------------------------------------------------------
void register_tests_feature_extraction()
{
  // Detectors:
  lstTests.emplace_back(
      "feature_extraction [640x480]: Harris (OpenCV)", benchmark_detectFeatures<featHarris>, 30);
  lstTests.emplace_back(
      "feature_extraction [640x480]: KLT (OpenCV)", benchmark_detectFeatures<featKLT>, 30);
  lstTests.emplace_back(
      "feature_extraction [640x480]: SIFT detect (OpenCV)", benchmark_detectFeatures<featSIFT>, 5);
  lstTests.emplace_back(
      "feature_extraction [640x480]: SURF (OpenCV)", benchmark_detectFeatures<featSURF>, 10);
  lstTests.emplace_back(
      "feature_extraction [640x480]: ORB (OpenCV)", benchmark_detectFeatures<featORB>, 10);
  lstTests.emplace_back(
      "feature_extraction [640x480]: FAST (OpenCV)", benchmark_detectFeatures<featFAST>, 100);

  lstTests.emplace_back(
      "feature_extraction [640x480]: AKAZE (OpenCV)", benchmark_detectFeatures<featAKAZE>, 5);
  lstTests.emplace_back(
      "feature_extraction [640x480]: LSD (OpenCV)", benchmark_detectFeatures<featLSD>, 5);

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
