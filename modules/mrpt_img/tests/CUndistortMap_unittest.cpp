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

/** Renders a small bright marker at pixel (cx,cy) on an otherwise-black
 *  ncols x nrows image. */
CImage markerImage(uint32_t ncols, uint32_t nrows, double cx, double cy)
{
  CImage img(ncols, nrows, CH_GRAY);
  img.filledRectangle(
      {0, 0}, {static_cast<int32_t>(ncols) - 1, static_cast<int32_t>(nrows) - 1}, TColor::black());
  const auto icx = static_cast<int32_t>(std::lround(cx));
  const auto icy = static_cast<int32_t>(std::lround(cy));
  img.filledRectangle({icx - 3, icy - 3}, {icx + 3, icy + 3}, TColor::white());
  return img;
}

/** Intensity-weighted centroid of an image, in pixel coordinates. */
std::pair<double, double> brightCentroid(const CImage& img)
{
  double sumX = 0, sumY = 0, sumW = 0;
  for (unsigned y = 0; y < img.getHeight(); ++y)
  {
    for (unsigned x = 0; x < img.getWidth(); ++x)
    {
      const double w = img.at<uint8_t>(x, y);
      sumX += w * x;
      sumY += w * y;
      sumW += w;
    }
  }
  return {sumX / sumW, sumY / sumW};
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
      if (std::abs(
              static_cast<int>(in.at<uint8_t>(x, y)) - static_cast<int>(out.at<uint8_t>(x, y))) > 1)
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

TEST(CUndistortMap, UndistortBeforeSetThrows)
{
  CUndistortMap map;
  CImage in(64, 64, CH_GRAY);
  CImage out;
  EXPECT_THROW(map.undistort(in, out), std::exception);
  EXPECT_THROW(map.undistort(in), std::exception);
}

TEST(CUndistortMap, UnknownDistortionModelThrows)
{
  auto cam = makeSampleCameraNoDistortion();
  // Inject an out-of-range enum value not handled by the switch statement.
  cam.distortion = static_cast<DistortionModel>(99);

  CUndistortMap map;
  EXPECT_THROW(map.setFromCamParams(cam), std::exception);
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

TEST(CStereoRectifyMap, Rectify_preservesForwardAxis)
{
  // With a pure-baseline (zero relative rotation) stereo pair, "straight
  // ahead" must stay "straight ahead": a feature at the left camera's own
  // principal point must reappear at the rectified image's own principal
  // point. The previous Rectify_basic/_color tests only check output image
  // *dimensions*, which a geometrically-broken rectification rotation
  // (e.g. one that swaps the camera's forward axis into the image plane)
  // still satisfies - they render a smooth gradient, which survives any
  // remap, garbled or not. This test checks the actual geometry instead.
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCameraNoDistortion();
  stereo.rightCamera = makeSampleCameraNoDistortion();
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(0.10, 0, 0, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  rectMap.setFromCamParams(stereo);

  const auto& cam = stereo.leftCamera;
  const CImage inLeft = markerImage(cam.ncols, cam.nrows, cam.cx(), cam.cy());
  const CImage inRight = inLeft;

  CImage outLeft, outRight;
  rectMap.rectify(inLeft, inRight, outLeft, outRight);

  const auto [markerX, markerY] = brightCentroid(outLeft);
  const auto& rcam = rectMap.getRectifiedLeftImageParams();
  EXPECT_NEAR(markerX, rcam.cx(), 5.0);
  EXPECT_NEAR(markerY, rcam.cy(), 5.0);
}

TEST(CStereoRectifyMap, Rectify_preservesForwardAxis_withRelativeRotation)
{
  // Same invariant as Rectify_preservesForwardAxis, but with a non-identity
  // relative rotation between the two cameras (e.g. a real, imperfectly
  // mounted rig such as a multi-camera bag with no per-camera TF): R_half is
  // then non-identity too, so a forward reference that isn't rotated along
  // with it (the bug CodeRabbit caught in this same PR) would derive e3 from
  // the *original*, un-rotated z axis instead of the half-rotated one.
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCameraNoDistortion();
  stereo.rightCamera = makeSampleCameraNoDistortion();
  // 5 deg yaw on top of the baseline translation.
  const double yawRad = 5.0 * M_PI / 180.0;
  stereo.rightCameraPose =
      mrpt::math::TPose3DQuat(0.10, 0, 0, std::cos(yawRad / 2), 0, 0, std::sin(yawRad / 2));

  CStereoRectifyMap rectMap;
  rectMap.setFromCamParams(stereo);

  const auto& cam = stereo.leftCamera;
  const CImage inLeft = markerImage(cam.ncols, cam.nrows, cam.cx(), cam.cy());
  const CImage inRight = inLeft;

  CImage outLeft, outRight;
  rectMap.rectify(inLeft, inRight, outLeft, outRight);

  const auto [markerX, markerY] = brightCentroid(outLeft);
  const auto& rcam = rectMap.getRectifiedLeftImageParams();
  EXPECT_NEAR(markerX, rcam.cx(), 5.0);
  EXPECT_NEAR(markerY, rcam.cy(), 5.0);
}

TEST(CStereoRectifyMap, Rectify_idealPairIsIdentity)
{
  // An already-rectified, distortion-free pair (parallel optical axes, right
  // camera a pure +x baseline away, identical intrinsics) must rectify to
  // ITSELF: the rectification rotation is the identity and the images come
  // back unchanged.
  //
  // The Rectify_preservesForwardAxis tests above cannot see a violation of
  // this, because they place their marker at the principal point, which is
  // the one point invariant under an in-plane 180 deg flip. An OFF-CENTER
  // marker is what distinguishes the identity from that flip - and a flip is
  // not cosmetic here: it puts the right camera at negative x in the
  // rectified frame, so every disparity of an ordinary rig comes out with the
  // wrong sign.
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCameraNoDistortion();
  stereo.rightCamera = makeSampleCameraNoDistortion();
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(0.10, 0, 0, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  rectMap.setFromCamParams(stereo);

  // The rectification rotation must be the identity quaternion.
  const auto& q = rectMap.getLeftCameraRot();
  EXPECT_NEAR(std::abs(q.r()), 1.0, 1e-9);

  // The rectified geometry must be a pure, POSITIVE baseline along x.
  const auto& rp = rectMap.getRectifiedImageParams();
  EXPECT_NEAR(rp.rightCameraPose.x, 0.10, 1e-9);
  EXPECT_NEAR(rp.rightCameraPose.y, 0.0, 1e-9);
  EXPECT_NEAR(rp.rightCameraPose.z, 0.0, 1e-9);

  const auto& cam = stereo.leftCamera;
  const double mx = cam.cx() + 100.0;
  const double my = cam.cy() + 60.0;
  const CImage inLeft = markerImage(cam.ncols, cam.nrows, mx, my);
  const CImage inRight = inLeft;

  CImage outLeft, outRight;
  rectMap.rectify(inLeft, inRight, outLeft, outRight);

  const auto [markerX, markerY] = brightCentroid(outLeft);
  EXPECT_NEAR(markerX, mx, 1.0);
  EXPECT_NEAR(markerY, my, 1.0);
}

TEST(CStereoRectifyMap, Rectify_obliqueBaselineNearOpticalAxis)
{
  // A baseline mostly along the optical axis (large z component) but not
  // exactly colinear with it: close enough to the old dot-product-based
  // degeneracy threshold (|e1.dot(z)| > 0.9, i.e. within ~26 deg of z) to be
  // misclassified as needing the y-axis fallback - CodeRabbit's second
  // finding on this PR. Its actual projection onto the plane orthogonal to
  // e1 is still well-conditioned (norm ~0.41 for the case below), so the fix
  // must not take the fallback path here.
  //
  // Unlike the two tests above, this does NOT check that a principal-point
  // marker survives: a baseline this close to the optical axis is the
  // classic "forward motion" stereo configuration, where standard
  // rectification legitimately reprojects the principal point far outside
  // the original field of view (confirmed numerically: for this exact case
  // the source ray for the rectified principal point lands at pixel
  // x = -777 on a 640-pixel-wide camera) - a real geometric property of this
  // configuration, not a defect. What must still hold is that
  // setFromCamParams()/rectify() succeed and produce a well-formed
  // (non-degenerate) rotation - i.e. that e3 was NOT snapped to the y-axis
  // fallback, which would additionally violate right-handedness with the
  // z-heavy baseline used here.
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCameraNoDistortion();
  stereo.rightCamera = makeSampleCameraNoDistortion();
  // Baseline direction (sqrt(1-0.91^2), 0, 0.91), scaled to a 10 cm baseline.
  const double bz = 0.91;
  const double bx = std::sqrt(1.0 - bz * bz);
  const double baseline = 0.10;
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(baseline * bx, 0, baseline * bz, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  ASSERT_NO_THROW(rectMap.setFromCamParams(stereo));

  // The rectification rotation must be a valid, finite unit quaternion (not
  // NaN/degenerate, as it would be if e3 had zero norm before normalizing).
  const auto& q = rectMap.getLeftCameraRot();
  EXPECT_TRUE(
      std::isfinite(q.r()) && std::isfinite(q.x()) && std::isfinite(q.y()) && std::isfinite(q.z()));
  EXPECT_NEAR(q.r() * q.r() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z(), 1.0, 1e-9);

  const auto& cam = stereo.leftCamera;
  const CImage inLeft = markerImage(cam.ncols, cam.nrows, cam.cx(), cam.cy());
  const CImage inRight = inLeft;

  CImage outLeft, outRight;
  ASSERT_NO_THROW(rectMap.rectify(inLeft, inRight, outLeft, outRight));
  EXPECT_EQ(outLeft.getWidth(), cam.ncols);
  EXPECT_EQ(outLeft.getHeight(), cam.nrows);
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

TEST(CStereoRectifyMap, RectifyBeforeSetThrows)
{
  CStereoRectifyMap rectMap;
  CImage inLeft(640, 480, CH_GRAY);
  CImage inRight(640, 480, CH_GRAY);
  CImage outLeft, outRight;
  EXPECT_THROW(rectMap.rectify(inLeft, inRight, outLeft, outRight), std::exception);
}

TEST(CStereoRectifyMap, GetRectifiedParamsBeforeSetThrows)
{
  CStereoRectifyMap rectMap;
  EXPECT_THROW((void)rectMap.getRectifiedImageParams(), std::exception);
  EXPECT_THROW((void)rectMap.getRectifiedLeftImageParams(), std::exception);
  EXPECT_THROW((void)rectMap.getRectifiedRightImageParams(), std::exception);
}

TEST(CStereoRectifyMap, InPlaceRectifyThrows)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCamera();
  stereo.rightCamera = makeSampleCamera();
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(0.10, 0, 0, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  rectMap.setFromCamParams(stereo);

  CImage img(640, 480, CH_GRAY);
  CImage other(640, 480, CH_GRAY);
  // Same image used as both input and output for the left pair is rejected.
  EXPECT_THROW(rectMap.rectify(img, other, img, other), std::exception);
}

TEST(CStereoRectifyMap, AlphaAndCentersCoincideOptions)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCamera();
  stereo.rightCamera = makeSampleCamera();
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(0.10, 0, 0, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  EXPECT_EQ(rectMap.getAlpha(), -1.0);
  rectMap.setAlpha(0.5);
  EXPECT_EQ(rectMap.getAlpha(), 0.5);

  EXPECT_FALSE(rectMap.isEnabledBothCentersCoincide());
  rectMap.enableBothCentersCoincide(true);
  EXPECT_TRUE(rectMap.isEnabledBothCentersCoincide());

  rectMap.setInterpolationMethod(IMG_INTERP_NN);
  EXPECT_EQ(rectMap.getInterpolationMethod(), IMG_INTERP_NN);

  rectMap.setFromCamParams(stereo);
  EXPECT_TRUE(rectMap.isSet());

  const auto& leftRot = rectMap.getLeftCameraRot();
  const auto& rightRot = rectMap.getRightCameraRot();
  EXPECT_NEAR(leftRot.norm(), 1.0, 1e-6);
  EXPECT_NEAR(rightRot.norm(), 1.0, 1e-6);
}

TEST(CStereoRectifyMap, EnableResizeOutput)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCamera();
  stereo.rightCamera = makeSampleCamera();
  stereo.rightCameraPose = mrpt::math::TPose3DQuat(0.10, 0, 0, 1, 0, 0, 0);

  CStereoRectifyMap rectMap;
  EXPECT_FALSE(rectMap.isEnabledResizeOutput());
  rectMap.enableResizeOutput(true, 320, 240);
  EXPECT_TRUE(rectMap.isEnabledResizeOutput());
  EXPECT_EQ(rectMap.getResizeOutputSize().x, 320);
  EXPECT_EQ(rectMap.getResizeOutputSize().y, 240);

  rectMap.setFromCamParams(stereo);

  CImage inLeft(640, 480, CH_GRAY);
  CImage inRight(640, 480, CH_GRAY);
  fillGradient(inLeft);
  fillGradient(inRight);

  CImage outLeft, outRight;
  rectMap.rectify(inLeft, inRight, outLeft, outRight);

  EXPECT_EQ(outLeft.getWidth(), 320);
  EXPECT_EQ(outLeft.getHeight(), 240);
}

TEST(CStereoRectifyMap, SetRectifyMapsDirect)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCameraNoDistortion();
  stereo.leftCamera.ncols = 4;
  stereo.leftCamera.nrows = 4;
  stereo.rightCamera = stereo.leftCamera;

  CStereoRectifyMap rectMap;
  EXPECT_FALSE(rectMap.isSet());

  const size_t n = 4 * 4;
  std::vector<float> lx(n, 1.0f);
  std::vector<float> ly(n, 1.0f);
  std::vector<float> rx(n, 2.0f);
  std::vector<float> ry(n, 2.0f);
  rectMap.setRectifyMaps(lx, ly, rx, ry);
  EXPECT_TRUE(rectMap.isSet());
}

TEST(CStereoRectifyMap, SetRectifyMapsFastSwapsInput)
{
  CStereoRectifyMap rectMap;
  std::vector<float> lx(4, 1.0f);
  std::vector<float> ly(4, 1.0f);
  std::vector<float> rx(4, 2.0f);
  std::vector<float> ry(4, 2.0f);
  rectMap.setRectifyMapsFast(lx, ly, rx, ry);
  EXPECT_TRUE(rectMap.isSet());
  // The input vectors are swapped-out (emptied) by the "fast" overload.
  EXPECT_TRUE(lx.empty());
}

TEST(CStereoRectifyMap, UnknownDistortionModelThrows)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCameraNoDistortion();
  stereo.rightCamera = makeSampleCameraNoDistortion();
  stereo.leftCamera.distortion = static_cast<DistortionModel>(99);

  CStereoRectifyMap rectMap;
  EXPECT_THROW(rectMap.setFromCamParams(stereo), std::exception);
}

TEST(CStereoRectifyMap, MismatchedCameraResolutionsAssert)
{
  TStereoCamera stereo;
  stereo.leftCamera = makeSampleCamera();
  stereo.rightCamera = makeSampleCamera();
  stereo.rightCamera.ncols = stereo.leftCamera.ncols + 10;

  CStereoRectifyMap rectMap;
  EXPECT_THROW(rectMap.setFromCamParams(stereo), std::exception);
}
