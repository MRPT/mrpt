/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
  CLASS: CFeatureExtraction
  FILE: CFeatureExtraction_LATCH.cpp
  AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include "vision-precomp.h"  // Precompiled headers
//
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/system/os.h>
#include <mrpt/vision/CFeatureExtraction.h>  // important import

// Universal include for all versions of OpenCV
#include <mrpt/3rdparty/do_opencv_includes.h>

#ifdef HAVE_OPENCV_NONFREE  // MRPT_HAS_OPENCV_NONFREE
#include <opencv2/nonfree/nonfree.hpp>
#endif

#ifdef HAVE_OPENCV_XFEATURES2D
#include <opencv2/xfeatures2d.hpp>
#endif
#ifdef HAVE_OPENCV_LINE_DESCRIPTOR
#include <opencv2/line_descriptor.hpp>
using namespace cv::line_descriptor;
#endif

using namespace mrpt::vision;
using namespace mrpt::img;
using namespace mrpt::math;
using namespace mrpt::img;
using namespace mrpt;
using namespace std;

#if defined(HAVE_OPENCV_XFEATURES2D) && defined(HAVE_OPENCV_LINE_DESCRIPTOR)
#define HAVE_OPENCV_WITH_LATCH 1
#else
#define HAVE_OPENCV_WITH_LATCH 0
#endif

void CFeatureExtraction::internal_computeLATCHDescriptors(
    const mrpt::img::CImage& in_img, CFeatureList& in_features)
{
  MRPT_START

  mrpt::system::CTimeLoggerEntry tle(profiler, "internal_computeLATCHDescriptors");

#if (!HAVE_OPENCV_WITH_LATCH)
  THROW_EXCEPTION("This function requires OpenCV modules: xfeatures2d,line_descriptor");
#else
  using namespace cv;

  if (in_features.empty()) return;

  const size_t n_feats = in_features.size();
  // Make sure we operate on a gray-scale version of the image:
  const CImage inImg_gray(in_img, FAST_REF_OR_CONVERT_TO_GRAY);

  // convert from CFeatureList to vector<KeyPoint>
  vector<KeyPoint> cv_feats(n_feats);
  for (size_t k = 0; k < n_feats; ++k)
  {
    KeyPoint& kp = cv_feats[k];
    kp.pt.x = in_features[k].keypoint.pt.x;
    kp.pt.y = in_features[k].keypoint.pt.y;
    kp.angle = in_features[k].orientation;
    kp.size = in_features[k].keypoint.octave;  // ?
  }                                            // end-for

  const Mat& cvImg = inImg_gray.asCvMatRef();
  Mat cv_descs;  // OpenCV descriptor output

  Ptr<xfeatures2d::LATCH> latch = xfeatures2d::LATCH::create(
      options.LATCHOptions.bytes, options.LATCHOptions.rotationInvariance,
      options.LATCHOptions.half_ssd_size);
  latch->compute(cvImg, cv_feats, cv_descs);

  // -----------------------------------------------------------------
  // MRPT Wrapping
  // -----------------------------------------------------------------
  int i = 0;
  for (auto& ft : in_features)
  {
    // Get the LATCH descriptor
    ft.descriptors.LATCH.emplace();
    ft.descriptors.LATCH->resize(cv_descs.cols);
    for (int m = 0; m < cv_descs.cols; ++m) (*ft.descriptors.LATCH)[m] = cv_descs.at<int>(i, m);

    i++;
  }

#endif
  MRPT_END
}  // end internal_computeLatchDescriptors
