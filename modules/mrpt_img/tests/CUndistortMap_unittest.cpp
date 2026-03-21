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

#include <gtest/gtest.h>
#include <mrpt/img/CImage.h>
#include <mrpt/img/CStereoRectifyMap.h>
#include <mrpt/img/CUndistortMap.h>
#include <mrpt/img/TStereoCamera.h>
#include <mrpt/random.h>
#include <mrpt/system/filesystem.h>
#include <test_mrpt_common.h>

using namespace mrpt::img;
using namespace std::string_literals;

namespace
{
TCamera makeSampleCamera()
{
  TCamera cam;
  cam.ncols = 640;
  cam.nrows = 480;
  cam.setIntrinsicParamsFromValues(800.0, 800.0, 320.0, 240.0);
  cam.distortion = DistortionModel::plumb_bob;
  cam.setDistortionPlumbBob(-0.28, 0.07, 0.0002, 0.0002, 0.0);
  return cam;
}

TCamera makeSampleCameraNoDistortion()
{
  TCamera cam;
  cam.ncols = 640;
  cam.nrows = 480;
  cam.setIntrinsicParamsFromValues(500.0, 500.0, 320.0, 240.0);
  cam.distortion = DistortionModel::none;
  return cam;
}

TCamera makeSampleCameraFisheye()
{
  TCamera cam;
  cam.ncols = 640;
  cam.nrows = 480;
  cam.setIntrinsicParamsFromValues(400.0, 400.0, 320.0, 240.0);
  cam.distortion = DistortionModel::kannala_brandt;
  cam.setDistortionKannalaBrandt(0.1, 0.01, 0.001, 0.0001);
  return cam;
}

void fillGradient(CImage& img)
{
  for (int y = 0; y < img.getHeight(); ++y)
  {
    auto* row = img.ptrLine<uint8_t>(y);
    for (int x = 0; x < img.getWidth(); ++x)
    {
      row[x] = static_cast<uint8_t>((x + y) % 256);
    }
  }
}

void fillGradientColor(CImage& img)
{
  for (int y = 0; y < img.getHeight(); ++y)
  {
    auto* row = img.ptrLine<uint8_t>(y);
    for (int x = 0; x < img.getWidth(); ++x)
    {
      row[x * 3 + 0] = static_cast<uint8_t>((x) % 256);
      row[x * 3 + 1] = static_cast<uint8_t>((y) % 256);
      row[x * 3 + 2] = static_cast<uint8_t>((x + y) % 256);
    }
  }
}

}  // namespace

// ===========================================================================
//  CUndistortMap tests
// ===========================================================================

TEST(CUndistortMap, SetFromCamParams)
{
  CUndistortMap map;
  EXPECT_FALSE(map.isSet());

  map.setFromCamParams(makeSampleCamera());
  EXPECT_TRUE(map.isSet());

  const auto& cp = map.getCameraParams();
  EXPECT_EQ(cp.ncols, 640U);
  EXPECT_EQ(cp.nrows, 480U);
}

TEST(CUndistortMap, Undistort_identity)
{
  // With no distortion, undistorted image should be very close to input.
  auto cam = makeSampleCameraNoDistortion();

  CUndistortMap map;
  map.setFromCamParams(cam);

  CImage in(static_cast<int>(cam.ncols), static_cast<int>(cam.nrows), CH_GRAY);
  fillGradient(in);

  CImage out;
  map.undistort(in, out);

  EXPECT_EQ(out.getWidth(), in.getWidth());
  EXPECT_EQ(out.getHeight(), in.getHeight());

  // Interior pixels should be identical (borders may differ due to remap boundary)
  int mismatches = 0;
  const int margin = 2;
  for (int y = margin; y < in.getHeight() - margin; ++y)
  {
    for (int x = margin; x < in.getWidth() - margin; ++x)
    {
      if (std::abs(static_cast<int>(in.at<uint8_t>(x, y)) -
                   static_cast<int>(out.at<uint8_t>(x, y))) > 1)
      {
        mismatches++;
      }
    }
  }
  EXPECT_EQ(mismatches, 0) << "Identity undistortion should preserve interior pixels";
}

TEST(CUndistortMap, Undistort_plumbbob_grayscale)
{
  auto cam = makeSampleCamera();

  CUndistortMap map;
  map.setFromCamParams(cam);

  CImage in(static_cast<int>(cam.ncols), static_cast<int>(cam.nrows), CH_GRAY);
  fillGradient(in);

  CImage out;
  map.undistort(in, out);

  EXPECT_EQ(out.getWidth(), in.getWidth());
  EXPECT_EQ(out.getHeight(), in.getHeight());

  // Center pixel should be approximately preserved (optical center)
  const int cx = static_cast<int>(cam.cx());
  const int cy = static_cast<int>(cam.cy());
  EXPECT_NEAR(
      static_cast<int>(in.at<uint8_t>(cx, cy)), static_cast<int>(out.at<uint8_t>(cx, cy)), 2)
      << "Center pixel should be approximately preserved";
}

TEST(CUndistortMap, Undistort_plumbbob_color)
{
  auto cam = makeSampleCamera();

  CUndistortMap map;
  map.setFromCamParams(cam);

  CImage in(static_cast<int>(cam.ncols), static_cast<int>(cam.nrows), CH_RGB);
  fillGradientColor(in);

  CImage out;
  map.undistort(in, out);

  EXPECT_EQ(out.getWidth(), in.getWidth());
  EXPECT_EQ(out.getHeight(), in.getHeight());
  EXPECT_TRUE(out.isColor());
}

TEST(CUndistortMap, Undistort_kannalaBrandt)
{
  auto cam = makeSampleCameraFisheye();

  CUndistortMap map;
  map.setFromCamParams(cam);

  CImage in(static_cast<int>(cam.ncols), static_cast<int>(cam.nrows), CH_GRAY);
  fillGradient(in);

  CImage out;
  map.undistort(in, out);

  EXPECT_EQ(out.getWidth(), in.getWidth());
  EXPECT_EQ(out.getHeight(), in.getHeight());
}

TEST(CUndistortMap, Undistort_inPlace)
{
  auto cam = makeSampleCamera();

  CUndistortMap map;
  map.setFromCamParams(cam);

  CImage img(static_cast<int>(cam.ncols), static_cast<int>(cam.nrows), CH_GRAY);
  fillGradient(img);

  map.undistort(img);

  EXPECT_EQ(img.getWidth(), static_cast<int>(cam.ncols));
  EXPECT_EQ(img.getHeight(), static_cast<int>(cam.nrows));
}

TEST(CUndistortMap, Undistort_fromFile)
{
  const auto tstImg =
      mrpt::UNITTEST_BASEDIR() + "/../../mrpt_examples_cpp/img_basic_example/frame_color.jpg"s;

  if (!mrpt::system::fileExists(tstImg))
  {
    GTEST_SKIP() << "Test image not found: " << tstImg;
  }

  CImage img;
  ASSERT_TRUE(img.loadFromFile(tstImg));

  auto cam = makeSampleCamera();
  cam.ncols = img.getWidth();
  cam.nrows = img.getHeight();
  cam.setIntrinsicParamsFromValues(
      static_cast<double>(img.getWidth()) * 1.2, static_cast<double>(img.getWidth()) * 1.2,
      static_cast<double>(img.getWidth()) * 0.5, static_cast<double>(img.getHeight()) * 0.5);

  CUndistortMap map;
  map.setFromCamParams(cam);

  CImage out;
  map.undistort(img, out);

  EXPECT_EQ(out.getWidth(), img.getWidth());
  EXPECT_EQ(out.getHeight(), img.getHeight());
}

// ===========================================================================
//  CImage::undistort() test
// ===========================================================================

TEST(CImage, Undistort)
{
  auto cam = makeSampleCamera();

  CImage in(static_cast<int>(cam.ncols), static_cast<int>(cam.nrows), CH_GRAY);
  fillGradient(in);

  CImage out;
  in.undistort(out, cam);

  EXPECT_EQ(out.getWidth(), in.getWidth());
  EXPECT_EQ(out.getHeight(), in.getHeight());
}

// ===========================================================================
//  CStereoRectifyMap tests
// ===========================================================================

TEST(CStereoRectifyMap, SetFromCamParams)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCamera();
  stereo.rightCamera = makeSampleCamera();
  // Baseline of 10cm along x
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(0.10, 0, 0, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  EXPECT_FALSE(rectMap.isSet());

  rectMap.setFromCamParams(stereo);
  EXPECT_TRUE(rectMap.isSet());

  const auto& rp = rectMap.getRectifiedImageParams();
  EXPECT_EQ(rp.leftCamera.ncols, 640U);
  EXPECT_EQ(rp.leftCamera.nrows, 480U);
}

TEST(CStereoRectifyMap, Rectify_basic)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCamera();
  stereo.rightCamera = makeSampleCamera();
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(0.10, 0, 0, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  rectMap.setFromCamParams(stereo);

  CImage inLeft(640, 480, CH_GRAY);
  CImage inRight(640, 480, CH_GRAY);
  fillGradient(inLeft);
  fillGradient(inRight);

  CImage outLeft, outRight;
  rectMap.rectify(inLeft, inRight, outLeft, outRight);

  EXPECT_EQ(outLeft.getWidth(), 640);
  EXPECT_EQ(outLeft.getHeight(), 480);
  EXPECT_EQ(outRight.getWidth(), 640);
  EXPECT_EQ(outRight.getHeight(), 480);
}

TEST(CStereoRectifyMap, Rectify_color)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCamera();
  stereo.rightCamera = makeSampleCamera();
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(0.10, 0, 0, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  rectMap.setFromCamParams(stereo);

  CImage inLeft(640, 480, CH_RGB);
  CImage inRight(640, 480, CH_RGB);
  fillGradientColor(inLeft);
  fillGradientColor(inRight);

  CImage outLeft, outRight;
  rectMap.rectify(inLeft, inRight, outLeft, outRight);

  EXPECT_EQ(outLeft.getWidth(), 640);
  EXPECT_TRUE(outLeft.isColor());
}
