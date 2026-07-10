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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/viz/CPointCloud.h>
#include <mrpt/viz/CPointCloudColoured.h>

#include <sstream>

using namespace mrpt::obs;
using namespace mrpt::img;
using namespace mrpt::math;

namespace
{
// Fill a WxH range image with a synthetic constant-depth plane, and set up
// consistent camera intrinsics, so that geometric methods have valid, testable
// data to work with.
void fillSyntheticScan(CObservation3DRangeScan& o, int W, int H, float depth = 2.0f)
{
  o.hasRangeImage = true;
  o.range_is_depth = true;
  o.rangeUnits = 0.001f;
  o.maxRange = 10.0f;
  o.rangeImage_setSize(H, W);
  for (int r = 0; r < H; r++)
    for (int c = 0; c < W; c++) o.rangeImage(r, c) = static_cast<uint16_t>(depth / o.rangeUnits);

  o.cameraParams.ncols = W;
  o.cameraParams.nrows = H;
  o.cameraParams.setIntrinsicParamsFromValues(250.0, 250.0, W / 2.0, H / 2.0);
  o.cameraParamsIntensity = o.cameraParams;
}
}  // namespace

TEST(CObservation3DRangeScan, DefaultStateDescription)
{
  CObservation3DRangeScan o;
  std::stringstream ss;
  o.getDescriptionAsText(ss);
  EXPECT_NE(ss.str().find("NO"), std::string::npos);
}

TEST(CObservation3DRangeScan, PopulatedDescriptionAndFlags)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 16, 12);

  o.resizePoints3DVectors(4);
  o.hasPoints3D = true;
  o.points3D_x[0] = 1;
  o.points3D_y[0] = 0;
  o.points3D_z[0] = 0;

  o.hasIntensityImage = true;
  o.intensityImage = CImage(16, 12, mrpt::img::CH_RGB);
  o.intensityImage.filledRectangle({0, 0}, {15, 11}, mrpt::img::TColor(100, 150, 200));

  o.hasConfidenceImage = true;
  o.confidenceImage = CImage(16, 12, mrpt::img::CH_GRAY);

  o.pixelLabels = std::make_shared<TPixelLabelInfo<1>>();
  o.pixelLabels->setSize(12, 16);
  o.pixelLabels->setLabelName(0, "floor");
  o.pixelLabels->setLabel(0, 0, 0);
  EXPECT_TRUE(o.hasPixelLabels());
  EXPECT_TRUE(o.pixelLabels->checkLabel(0, 0, 0));

  o.rangeImageOtherLayers["SECOND"] = o.rangeImage;

  std::stringstream ss;
  o.getDescriptionAsText(ss);
  const std::string txt = ss.str();
  EXPECT_NE(txt.find("YES"), std::string::npos);
  EXPECT_NE(txt.find("floor"), std::string::npos);
  EXPECT_NE(txt.find("Additional rangeImage layer"), std::string::npos);
  EXPECT_EQ(o.getScanSize(), 4u);
}

TEST(CObservation3DRangeScan, SerializationRoundtrip)
{
  CObservation3DRangeScan o1;
  fillSyntheticScan(o1, 16, 8);
  o1.resizePoints3DVectors(3);
  o1.hasPoints3D = true;
  o1.points3D_x[1] = 5;
  o1.points3D_idxs_x = {1, 2, 3};
  o1.points3D_idxs_y = {1, 2, 3};
  o1.hasIntensityImage = true;
  o1.intensityImage = CImage(16, 8, mrpt::img::CH_RGB);
  o1.sensorPose = mrpt::poses::CPose3D(1, 2, 3, 0.1, 0.2, 0.3);
  o1.stdError = 0.02f;
  o1.pixelLabels = std::make_shared<TPixelLabelInfo<1>>();
  o1.pixelLabels->setSize(8, 16);
  o1.pixelLabels->setLabelName(2, "wall");

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << o1;
  buf.Seek(0);

  CObservation3DRangeScan o2;
  arch >> o2;

  EXPECT_EQ(o2.hasRangeImage, o1.hasRangeImage);
  EXPECT_EQ(o2.rangeImage.rows(), o1.rangeImage.rows());
  EXPECT_EQ(o2.rangeImage.cols(), o1.rangeImage.cols());
  EXPECT_EQ(o2.hasPoints3D, o1.hasPoints3D);
  EXPECT_EQ(o2.points3D_x.size(), o1.points3D_x.size());
  EXPECT_FLOAT_EQ(o2.points3D_x[1], o1.points3D_x[1]);
  EXPECT_EQ(o2.hasIntensityImage, o1.hasIntensityImage);
  EXPECT_NEAR(o2.sensorPose.x(), o1.sensorPose.x(), 1e-9);
  EXPECT_FLOAT_EQ(o2.stdError, o1.stdError);
  ASSERT_TRUE(o2.hasPixelLabels());
  EXPECT_EQ(o2.pixelLabels->getLabelName(2), "wall");
}

TEST(CObservation3DRangeScan, Swap)
{
  CObservation3DRangeScan a, b;
  fillSyntheticScan(a, 8, 8, 1.0f);
  fillSyntheticScan(b, 4, 4, 2.0f);
  a.sensorLabel = "A";
  b.sensorLabel = "B";

  a.swap(b);
  EXPECT_EQ(a.sensorLabel, "B");
  EXPECT_EQ(b.sensorLabel, "A");
  EXPECT_EQ(a.rangeImage.cols(), 4);
  EXPECT_EQ(b.rangeImage.cols(), 8);
}

TEST(CObservation3DRangeScan, DoDepthAndIntensityCamerasCoincide)
{
  CObservation3DRangeScan o;
  // Default value is the special rotation that means "coincide":
  EXPECT_TRUE(o.doDepthAndIntensityCamerasCoincide());
  o.relativePoseIntensityWRTDepth = mrpt::poses::CPose3D(0.1, 0, 0, 0, 0, 0);
  EXPECT_FALSE(o.doDepthAndIntensityCamerasCoincide());
}

TEST(CObservation3DRangeScan, RangeImageSetSizeAndGetScanSize)
{
  CObservation3DRangeScan o;
  o.rangeImage_setSize(10, 20);
  EXPECT_EQ(o.rangeImage.rows(), 10);
  EXPECT_EQ(o.rangeImage.cols(), 20);
  EXPECT_EQ(o.getScanSize(), 0u);  // hasPoints3D still false, points3D empty

  o.resizePoints3DVectors(50);
  EXPECT_EQ(o.points3D_x.size(), 50u);
  EXPECT_EQ(o.getScanSize(), 50u);
}

TEST(CObservation3DRangeScan, UnprojectIntoSimpleNoColorSSE2Path)
{
  CObservation3DRangeScan o;
  // Width multiple of 8 to exercise the SSE2 optimized path.
  fillSyntheticScan(o, 16, 8, 2.0f);

  mrpt::viz::CPointCloud::Ptr pc = mrpt::viz::CPointCloud::Create();
  o.unprojectInto(*pc);
  EXPECT_EQ(pc->size(), 16u * 8u);
}

TEST(CObservation3DRangeScan, UnprojectIntoScalarPathWithDecimation)
{
  CObservation3DRangeScan o;
  // Width NOT multiple of 8: forces the scalar (non-SSE2) code path.
  fillSyntheticScan(o, 12, 6, 2.0f);

  mrpt::viz::CPointCloud::Ptr pc = mrpt::viz::CPointCloud::Create();
  T3DPointsProjectionParams pp;
  pp.decimation = 2;
  o.unprojectInto(*pc, pp);
  EXPECT_EQ(pc->size(), 6u * 3u);
}

TEST(CObservation3DRangeScan, UnprojectIntoEmptyRangeImage)
{
  CObservation3DRangeScan o;  // hasRangeImage == false
  mrpt::viz::CPointCloud::Ptr pc = mrpt::viz::CPointCloud::Create();
  pc->insertPoint(mrpt::math::TPoint3Df(1, 2, 3));
  o.unprojectInto(*pc);
  EXPECT_EQ(pc->size(), 0u);
}

TEST(CObservation3DRangeScan, UnprojectIntoWithColorDirectCorrespondence)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 16, 8, 2.0f);
  o.hasIntensityImage = true;
  o.intensityImage = CImage(16, 8, mrpt::img::CH_RGB);
  o.intensityImage.filledRectangle({0, 0}, {15, 7}, mrpt::img::TColor(10, 20, 30));

  mrpt::viz::CPointCloudColoured::Ptr pc = mrpt::viz::CPointCloudColoured::Create();
  T3DPointsProjectionParams pp;
  pp.takeIntoAccountSensorPoseOnRobot = true;
  o.unprojectInto(*pc, pp);
  EXPECT_EQ(pc->size(), 16u * 8u);
}

TEST(CObservation3DRangeScan, UnprojectIntoWithColorReprojected)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 16, 8, 2.0f);
  o.hasIntensityImage = true;
  o.intensityImage = CImage(16, 8, mrpt::img::CH_RGB);
  // Force the "not directly corresponding" branch:
  o.relativePoseIntensityWRTDepth = mrpt::poses::CPose3D(0.01, 0, 0, 0, 0, 0);

  mrpt::viz::CPointCloudColoured::Ptr pc = mrpt::viz::CPointCloudColoured::Create();
  T3DPointsProjectionParams pp;
  pp.onlyPointsWithIntensityColor = true;
  o.unprojectInto(*pc, pp);
  // At least run without throwing; some points may be filtered out:
  EXPECT_LE(pc->size(), 16u * 8u);
}

TEST(CObservation3DRangeScan, UnprojectIntoOrganizedAndRobotPose)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 16, 8, 2.0f);

  mrpt::viz::CPointCloud::Ptr pc = mrpt::viz::CPointCloud::Create();
  T3DPointsProjectionParams pp;
  pp.MAKE_ORGANIZED = true;
  pp.takeIntoAccountSensorPoseOnRobot = true;
  pp.robotPoseInTheWorld = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);
  o.unprojectInto(*pc, pp);
  EXPECT_EQ(pc->size(), 16u * 8u);
}

TEST(CObservation3DRangeScan, UnprojectIntoWithRangeFilterMasks)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 16, 8, 2.0f);

  CMatrixF minMask(8, 16);
  minMask.setConstant(1.0f);  // valid ranges must be >= 1m

  mrpt::viz::CPointCloud::Ptr pc = mrpt::viz::CPointCloud::Create();
  TRangeImageFilterParams fp;
  fp.rangeMask_min = &minMask;
  fp.mark_invalid_ranges = true;
  o.unprojectInto(*pc, T3DPointsProjectionParams(), fp);
  EXPECT_EQ(pc->size(), 16u * 8u);  // depth=2.0 > 1.0, all pass
}

TEST(CObservation3DRangeScan, UnprojectIntoLayer)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 16, 8, 2.0f);
  o.rangeImageOtherLayers["SECOND"] = o.rangeImage;

  mrpt::viz::CPointCloud::Ptr pc = mrpt::viz::CPointCloud::Create();
  T3DPointsProjectionParams pp;
  pp.layer = "SECOND";
  o.unprojectInto(*pc, pp);
  EXPECT_EQ(pc->size(), 16u * 8u);
}

TEST(CObservation3DRangeScan, ConvertTo2DScanDefault)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 32, 16, 3.0f);

  CObservation2DRangeScan scan2d;
  T3DPointsTo2DScanParams params;
  params.sensorLabel = "fake2d";
  o.convertTo2DScan(scan2d, params);
  EXPECT_GT(scan2d.getScanSize(), 0u);
  EXPECT_EQ(scan2d.sensorLabel, "fake2d");
}

TEST(CObservation3DRangeScan, ConvertTo2DScanOriginSensorPose)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 32, 16, 3.0f);

  CObservation2DRangeScan scan2d;
  T3DPointsTo2DScanParams params;
  params.sensorLabel = "fake2d_origin";
  params.use_origin_sensor_pose = true;
  o.convertTo2DScan(scan2d, params);
  EXPECT_GT(scan2d.getScanSize(), 0u);
}

TEST(CObservation3DRangeScan, RangeImageAsImageStaticAndMember)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 16, 8, 2.0f);

  // Static method, explicit min/max:
  CImage img1 = CObservation3DRangeScan::rangeImageAsImage(o.rangeImage, 0.0f, 5.0f, o.rangeUnits);
  EXPECT_EQ(img1.getWidth(), 16u);
  EXPECT_EQ(img1.getHeight(), 8u);

  // Static method, auto max (val_max=0):
  CImage img2 = CObservation3DRangeScan::rangeImageAsImage(o.rangeImage, 0.0f, 0.0f, o.rangeUnits);
  EXPECT_EQ(img2.getWidth(), 16u);

  // Member method, default args:
  CImage img3 = o.rangeImage_getAsImage();
  EXPECT_EQ(img3.getWidth(), 16u);

  // Member method, additional layer:
  o.rangeImageOtherLayers["SECOND"] = o.rangeImage;
  CImage img4 = o.rangeImage_getAsImage(
      mrpt::img::cmJET, std::nullopt, std::nullopt, std::optional<std::string>("SECOND"));
  EXPECT_EQ(img4.getWidth(), 16u);
}

TEST(CObservation3DRangeScan, ExternalsAsTextFlag)
{
  const bool orig = CObservation3DRangeScan::EXTERNALS_AS_TEXT();
  CObservation3DRangeScan::EXTERNALS_AS_TEXT(true);
  EXPECT_TRUE(CObservation3DRangeScan::EXTERNALS_AS_TEXT());
  CObservation3DRangeScan::EXTERNALS_AS_TEXT(false);
  EXPECT_FALSE(CObservation3DRangeScan::EXTERNALS_AS_TEXT());
  CObservation3DRangeScan::EXTERNALS_AS_TEXT(orig);
}

TEST(CObservation3DRangeScan, ExternalStoragePointsAndRangeImageRoundtrip)
{
  const std::string tmpDir = mrpt::system::getTempFileName() + "_dir";
  mrpt::system::createDirectory(tmpDir);

  CObservation3DRangeScan o;
  fillSyntheticScan(o, 8, 4, 2.5f);
  o.resizePoints3DVectors(3);
  o.hasPoints3D = true;
  o.points3D_x[0] = 1.5f;

  ASSERT_FALSE(o.points3D_isExternallyStored());
  o.points3D_convertToExternalStorage("pts3d", tmpDir);
  EXPECT_TRUE(o.points3D_isExternallyStored());
  EXPECT_TRUE(o.points3D_x.empty());  // memory released after externalization

  ASSERT_FALSE(o.rangeImage_isExternallyStored());
  o.rangeImage_convertToExternalStorage("ri", tmpDir);
  EXPECT_TRUE(o.rangeImage_isExternallyStored());
  EXPECT_EQ(o.rangeImage.size(), 0u);  // memory released

  o.unload();  // should be a no-op given the fields are already unloaded

  // Now force a (re)load:
  const std::string savedDir = CImage::getImagesPathBase();
  CImage::setImagesPathBase(tmpDir);
  o.load();
  CImage::setImagesPathBase(savedDir);

  EXPECT_EQ(o.points3D_x.size(), 3u);
  EXPECT_FLOAT_EQ(o.points3D_x[0], 1.5f);
  EXPECT_EQ(o.rangeImage.rows(), 4);
  EXPECT_EQ(o.rangeImage.cols(), 8);

  mrpt::system::deleteFilesInDirectory(tmpDir, true /*delete dir too*/);
}

TEST(CObservation3DRangeScan, ExternalStorageAsTextRoundtrip)
{
  const std::string tmpDir = mrpt::system::getTempFileName() + "_dirtxt";
  mrpt::system::createDirectory(tmpDir);

  CObservation3DRangeScan::EXTERNALS_AS_TEXT(true);

  CObservation3DRangeScan o;
  fillSyntheticScan(o, 6, 3, 1.5f);
  o.resizePoints3DVectors(2);
  o.hasPoints3D = true;
  o.points3D_y[1] = 4.25f;

  o.points3D_convertToExternalStorage("pts3d_txt", tmpDir);
  o.rangeImage_convertToExternalStorage("ri_txt", tmpDir);

  const std::string savedDir = CImage::getImagesPathBase();
  CImage::setImagesPathBase(tmpDir);
  o.load();
  CImage::setImagesPathBase(savedDir);

  EXPECT_EQ(o.points3D_y.size(), 2u);
  EXPECT_NEAR(o.points3D_y[1], 4.25f, 1e-3f);
  EXPECT_EQ(o.rangeImage.rows(), 3);

  CObservation3DRangeScan::EXTERNALS_AS_TEXT(false);
  mrpt::system::deleteFilesInDirectory(tmpDir, true /*delete dir too*/);
}

TEST(CObservation3DRangeScan, RecoverCameraCalibrationParameters)
{
  CObservation3DRangeScan o;
  const int W = 60;
  const int H = 60;
  fillSyntheticScan(o, W, H, 2.0f);

  // Generate a consistent 3D point cloud with the exact same intrinsics used
  // as the optimizer's initial guess, so LM converges to (near) zero error:
  mrpt::viz::CPointCloud::Ptr pc = mrpt::viz::CPointCloud::Create();
  T3DPointsProjectionParams pp;
  pp.MAKE_ORGANIZED = true;
  o.unprojectInto(*pc, pp);

  // Copy the point cloud into the observation's own points3D_* buffers, as
  // required by recoverCameraCalibrationParameters():
  o.resizePoints3DVectors(pc->size());
  o.hasPoints3D = true;
  for (size_t i = 0; i < pc->size(); i++)
  {
    o.points3D_x[i] = pc->getPoint3Df(i).x;
    o.points3D_y[i] = pc->getPoint3Df(i).y;
    o.points3D_z[i] = pc->getPoint3Df(i).z;
  }

  mrpt::img::TCamera outCam;
  const double err = CObservation3DRangeScan::recoverCameraCalibrationParameters(o, outCam, 0.0);
  EXPECT_TRUE(std::isfinite(err));
  EXPECT_GE(err, 0.0);
  EXPECT_NEAR(outCam.fx(), 250.0, 5.0);
  EXPECT_NEAR(outCam.fy(), 250.0, 5.0);
}

TEST(CObservation3DRangeScan, GetUnprojLutIsConsistent)
{
  CObservation3DRangeScan o;
  fillSyntheticScan(o, 8, 8, 1.0f);
  const auto& lut1 = o.get_unproj_lut();
  const auto& lut2 = o.get_unproj_lut();
  EXPECT_EQ(lut1.Kxs.size(), lut2.Kxs.size());
  EXPECT_EQ(lut1.Kxs.size(), static_cast<size_t>(8 * 8));
}
