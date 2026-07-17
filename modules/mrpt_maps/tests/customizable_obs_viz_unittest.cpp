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
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/customizable_obs_viz.h>
#include <mrpt/viz/CPlanarLaserScan.h>

using namespace mrpt::obs;

// =========================================================================
//  VisualizationParameters / PointCloudRecoloringParameters ini round trips
// =========================================================================

TEST(CustomizableObsViz, VisualizationParametersIniRoundTrip)
{
  VisualizationParameters src;
  src.showAxis = false;
  src.axisTickFrequency = 2.5;
  src.axisLimits = 15.0;
  src.axisTickTextSize = 0.2;
  src.colorFromRGBimage = false;
  src.pointSize = 7.0;
  src.drawSensorPose = false;
  src.sensorPoseScale = 0.5;
  src.showSurfaceIn2Dscans = false;
  src.showPointsIn2Dscans = false;
  src.onlyPointsWithColor = true;
  src.surface2DscansColor = mrpt::img::TColor(10, 20, 30, 40);
  src.points2DscansColor = mrpt::img::TColor(50, 60, 70, 80);
  src.coloring.colorizeByField = "intensity";
  src.coloring.invertColorMapping = true;
  src.coloring.colorMap = mrpt::img::cmGRAYSCALE;

  mrpt::config::CConfigFileMemory cfg;
  src.save_to_ini_file(cfg, "TestSection");

  VisualizationParameters dst;
  dst.load_from_ini_file(cfg, "TestSection");

  EXPECT_EQ(dst.showAxis, src.showAxis);
  EXPECT_DOUBLE_EQ(dst.axisTickFrequency, src.axisTickFrequency);
  EXPECT_DOUBLE_EQ(dst.axisLimits, src.axisLimits);
  EXPECT_DOUBLE_EQ(dst.axisTickTextSize, src.axisTickTextSize);
  EXPECT_EQ(dst.colorFromRGBimage, src.colorFromRGBimage);
  EXPECT_DOUBLE_EQ(dst.pointSize, src.pointSize);
  EXPECT_EQ(dst.drawSensorPose, src.drawSensorPose);
  EXPECT_DOUBLE_EQ(dst.sensorPoseScale, src.sensorPoseScale);
  EXPECT_EQ(dst.showSurfaceIn2Dscans, src.showSurfaceIn2Dscans);
  EXPECT_EQ(dst.showPointsIn2Dscans, src.showPointsIn2Dscans);
  EXPECT_EQ(dst.onlyPointsWithColor, src.onlyPointsWithColor);
  EXPECT_EQ(dst.surface2DscansColor.R, src.surface2DscansColor.R);
  EXPECT_EQ(dst.surface2DscansColor.A, src.surface2DscansColor.A);
  EXPECT_EQ(dst.points2DscansColor.G, src.points2DscansColor.G);
  EXPECT_EQ(dst.coloring.colorizeByField, src.coloring.colorizeByField);
  EXPECT_EQ(dst.coloring.invertColorMapping, src.coloring.invertColorMapping);
  EXPECT_EQ(dst.coloring.colorMap, src.coloring.colorMap);
}

TEST(CustomizableObsViz, PointCloudRecoloringParametersIniRoundTripWithBounds)
{
  PointCloudRecoloringParameters src;
  src.colorizeByField = "z";
  src.colorMapMinCoord = 1.5f;
  src.colorMapMaxCoord = 9.5f;

  mrpt::config::CConfigFileMemory cfg;
  src.save_to_ini_file(cfg, "Coloring");

  PointCloudRecoloringParameters dst;
  dst.load_from_ini_file(cfg, "Coloring");

  ASSERT_TRUE(dst.colorMapMinCoord.has_value());
  ASSERT_TRUE(dst.colorMapMaxCoord.has_value());
  EXPECT_FLOAT_EQ(*dst.colorMapMinCoord, 1.5f);
  EXPECT_FLOAT_EQ(*dst.colorMapMaxCoord, 9.5f);
}

TEST(CustomizableObsViz, PointCloudRecoloringParametersIniRoundTripWithoutBounds)
{
  PointCloudRecoloringParameters src;
  src.colorizeByField = "ring";

  mrpt::config::CConfigFileMemory cfg;
  src.save_to_ini_file(cfg, "Coloring");

  PointCloudRecoloringParameters dst;
  dst.load_from_ini_file(cfg, "Coloring");

  EXPECT_EQ(dst.colorizeByField, "ring");
  EXPECT_FALSE(dst.colorMapMinCoord.has_value());
  EXPECT_FALSE(dst.colorMapMaxCoord.has_value());
}

// =========================================================================
//  recolorize3Dpc()
// =========================================================================

TEST(CustomizableObsViz, RecolorizeNullOriginalPtsIsNoOp)
{
  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->resize(3);

  PointCloudRecoloringParameters p;
  p.colorizeByField = "z";

  EXPECT_NO_THROW(recolorize3Dpc(pnts, nullptr, p));
}

TEST(CustomizableObsViz, RecolorizeNoColorMapIsNoOp)
{
  mrpt::maps::CSimplePointsMap map;
  map.insertPoint(1.0f, 2.0f, 3.0f);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(&map);

  PointCloudRecoloringParameters p;
  p.colorizeByField = "z";
  p.colorMap = mrpt::img::cmNONE;

  const auto colorBefore = pnts->getPointColor(0);
  recolorize3Dpc(pnts, &map, p);
  EXPECT_EQ(pnts->getPointColor(0), colorBefore);
}

TEST(CustomizableObsViz, RecolorizeByRgbUint8Fields)
{
  mrpt::maps::CGenericPointsMap map;
  map.registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8);
  map.registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8);
  map.registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8);

  const std::vector<std::array<uint8_t, 3>> colors = {
      { 10,  20, 30},
      {200, 100, 50}
  };
  for (const auto& c : colors)
  {
    map.insertPointFast(0, 0, 0);
    map.insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8, c[0]);
    map.insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8, c[1]);
    map.insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8, c[2]);
  }

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(&map);

  PointCloudRecoloringParameters p;
  p.colorizeByField = "rgb";

  recolorize3Dpc(pnts, &map, p);

  ASSERT_EQ(pnts->size(), colors.size());
  for (size_t i = 0; i < colors.size(); i++)
  {
    const auto col = pnts->getPointColor(i);
    EXPECT_EQ(col.R, colors[i][0]);
    EXPECT_EQ(col.G, colors[i][1]);
    EXPECT_EQ(col.B, colors[i][2]);
  }
}

TEST(CustomizableObsViz, RecolorizeByRgbFloatFields)
{
  mrpt::maps::CGenericPointsMap map;
  map.registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Rf);
  map.registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gf);
  map.registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bf);

  map.insertPointFast(0, 0, 0);
  map.insertPointField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Rf, 0.25f);
  map.insertPointField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gf, 0.5f);
  map.insertPointField_float(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bf, 0.75f);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(&map);

  PointCloudRecoloringParameters p;
  p.colorizeByField = "rgbf";

  recolorize3Dpc(pnts, &map, p);

  float r = 0, g = 0, b = 0;
  pnts->getPointColor_fast(0, r, g, b);
  EXPECT_NEAR(r, 0.25f, 0.01f);
  EXPECT_NEAR(g, 0.5f, 0.01f);
  EXPECT_NEAR(b, 0.75f, 0.01f);
}

TEST(CustomizableObsViz, RecolorizeRgbMissingFieldsIsNoOp)
{
  mrpt::maps::CSimplePointsMap map;
  map.insertPoint(1.0f, 2.0f, 3.0f);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(&map);
  const auto colorBefore = pnts->getPointColor(0);

  PointCloudRecoloringParameters p;
  p.colorizeByField = "rgb";  // Fields not present in a plain CSimplePointsMap.

  recolorize3Dpc(pnts, &map, p);
  EXPECT_EQ(pnts->getPointColor(0), colorBefore);
}

TEST(CustomizableObsViz, RecolorizeRgbMismatchedLengthIsSkipped)
{
  mrpt::maps::CGenericPointsMap map;
  map.registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8);
  map.registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8);
  map.registerField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8);
  map.insertPointFast(0, 0, 0);
  map.insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Ru8, 99);
  map.insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Gu8, 99);
  map.insertPointField_uint8(mrpt::maps::CPointsMap::POINT_FIELD_COLOR_Bu8, 99);

  // Deliberately mismatched size: the cloud has 2 points, the map only 1.
  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->resize(2);
  pnts->setPointColor_u8_fast(0, 1, 2, 3);
  pnts->setPointColor_u8_fast(1, 4, 5, 6);

  PointCloudRecoloringParameters p;
  p.colorizeByField = "rgb";

  recolorize3Dpc(pnts, &map, p);

  const auto col0 = pnts->getPointColor(0);
  EXPECT_EQ(col0.R, 1);
  EXPECT_EQ(col0.G, 2);
  EXPECT_EQ(col0.B, 3);
}

namespace
{
mrpt::maps::CGenericPointsMap::Ptr buildGenericMapForColoring(size_t n)
{
  auto map = mrpt::maps::CGenericPointsMap::Create();
  for (size_t i = 0; i < n; i++) map->insertPointFast(static_cast<float>(i), 0, 0);
  return map;
}
}  // namespace

TEST(CustomizableObsViz, RecolorizeByGenericFloatFieldWithOutlierRejection)
{
  auto map = buildGenericMapForColoring(20);
  const std::string field = "test_field_float_outlier";
  map->registerField_float(field);
  auto* buf = map->getPointsBufferRef_float_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<float>(i);
  (*buf)[buf->size() - 1] = 10000.0f;  // outlier

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(map.get());

  PointCloudRecoloringParameters p;
  p.colorizeByField = field;
  p.outlierRejectionPercentile = 0.1f;

  recolorize3Dpc(pnts, map.get(), p);

  const auto colFirst = pnts->getPointColor(0);
  const auto colMid = pnts->getPointColor(buf->size() / 2);
  EXPECT_NE(colFirst, colMid);
}

TEST(CustomizableObsViz, RecolorizeByGenericDoubleFieldWithOutlierRejection)
{
  auto map = buildGenericMapForColoring(20);
  const std::string field = "test_field_double_outlier";
  map->registerField_double(field);
  auto* buf = map->getPointsBufferRef_double_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<double>(i);
  (*buf)[buf->size() - 1] = 10000.0;  // outlier

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(map.get());

  PointCloudRecoloringParameters p;
  p.colorizeByField = field;
  p.outlierRejectionPercentile = 0.1f;

  recolorize3Dpc(pnts, map.get(), p);

  const auto colFirst = pnts->getPointColor(0);
  const auto colMid = pnts->getPointColor(buf->size() / 2);
  EXPECT_NE(colFirst, colMid);
}

TEST(CustomizableObsViz, RecolorizeByGenericUint8FieldWithOutlierRejection)
{
  auto map = buildGenericMapForColoring(20);
  const std::string field = "test_field_u8_outlier";
  map->registerField_uint8(field);
  auto* buf = map->getPointsBufferRef_uint8_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<uint8_t>(i);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(map.get());

  PointCloudRecoloringParameters p;
  p.colorizeByField = field;
  p.outlierRejectionPercentile = 0.05f;

  recolorize3Dpc(pnts, map.get(), p);

  const auto colFirst = pnts->getPointColor(0);
  const auto colLast = pnts->getPointColor(buf->size() - 1);
  EXPECT_NE(colFirst, colLast);
}

TEST(CustomizableObsViz, RecolorizeByGenericUint16FieldWithOutlierRejection)
{
  auto map = buildGenericMapForColoring(20);
  const std::string field = "test_field_u16_outlier";
  map->registerField_uint16(field);
  auto* buf = map->getPointsBufferRef_uint16_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<uint16_t>(i * 10);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(map.get());

  PointCloudRecoloringParameters p;
  p.colorizeByField = field;
  p.outlierRejectionPercentile = 0.05f;

  recolorize3Dpc(pnts, map.get(), p);

  const auto colFirst = pnts->getPointColor(0);
  const auto colLast = pnts->getPointColor(buf->size() - 1);
  EXPECT_NE(colFirst, colLast);
}

TEST(CustomizableObsViz, RecolorizeByGenericUint32FieldWithOutlierRejection)
{
  auto map = buildGenericMapForColoring(20);
  const std::string field = "test_field_u32_outlier";
  map->registerField_uint32(field);
  auto* buf = map->getPointsBufferRef_uint32_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<uint32_t>(i * 100);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(map.get());

  PointCloudRecoloringParameters p;
  p.colorizeByField = field;
  p.outlierRejectionPercentile = 0.05f;

  recolorize3Dpc(pnts, map.get(), p);

  const auto colFirst = pnts->getPointColor(0);
  const auto colLast = pnts->getPointColor(buf->size() - 1);
  EXPECT_NE(colFirst, colLast);
}

TEST(CustomizableObsViz, RecolorizeUnknownFieldIsNoOp)
{
  mrpt::maps::CSimplePointsMap map;
  map.insertPoint(1.0f, 2.0f, 3.0f);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(&map);
  const auto colorBefore = pnts->getPointColor(0);

  PointCloudRecoloringParameters p;
  p.colorizeByField = "this_field_does_not_exist";

  recolorize3Dpc(pnts, &map, p);
  EXPECT_EQ(pnts->getPointColor(0), colorBefore);
}

TEST(CustomizableObsViz, RecolorizeGenericFieldMismatchedLengthIsSkipped)
{
  auto map = buildGenericMapForColoring(3);
  const std::string field = "test_field_mismatch";
  map->registerField_float(field);
  auto* buf = map->getPointsBufferRef_float_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<float>(i);

  // pnts has a different point count than the map (5 vs 3).
  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->resize(5);
  pnts->setPointColor_u8_fast(0, 7, 8, 9);
  const auto colorBefore = pnts->getPointColor(0);

  PointCloudRecoloringParameters p;
  p.colorizeByField = field;

  recolorize3Dpc(pnts, map.get(), p);
  EXPECT_EQ(pnts->getPointColor(0), colorBefore);
}

TEST(CustomizableObsViz, RecolorizeInvertColorMapping)
{
  auto map = buildGenericMapForColoring(5);
  const std::string field = "test_field_invert";
  map->registerField_float(field);
  auto* buf = map->getPointsBufferRef_float_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<float>(i);

  auto pntsNormal = mrpt::viz::CPointCloudColoured::Create();
  pntsNormal->loadFromPointsMap(map.get());
  PointCloudRecoloringParameters pNormal;
  pNormal.colorizeByField = field;
  recolorize3Dpc(pntsNormal, map.get(), pNormal);

  const std::string fieldInv = "test_field_invert_b";
  map->registerField_float(fieldInv);
  auto* bufInv = map->getPointsBufferRef_float_field(fieldInv);
  ASSERT_NE(bufInv, nullptr);
  for (size_t i = 0; i < bufInv->size(); i++) (*bufInv)[i] = static_cast<float>(i);

  auto pntsInverted = mrpt::viz::CPointCloudColoured::Create();
  pntsInverted->loadFromPointsMap(map.get());
  PointCloudRecoloringParameters pInverted;
  pInverted.colorizeByField = fieldInv;
  pInverted.invertColorMapping = true;
  recolorize3Dpc(pntsInverted, map.get(), pInverted);

  // With inversion, the color at the low end and high end should swap.
  EXPECT_EQ(pntsNormal->getPointColor(0), pntsInverted->getPointColor(4));
  EXPECT_EQ(pntsNormal->getPointColor(4), pntsInverted->getPointColor(0));
}

TEST(CustomizableObsViz, RecolorizeZeroPercentileSkipsTrimming)
{
  auto map = buildGenericMapForColoring(5);
  const std::string field = "test_field_zero_pct";
  map->registerField_float(field);
  auto* buf = map->getPointsBufferRef_float_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<float>(i);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(map.get());

  PointCloudRecoloringParameters p;
  p.colorizeByField = field;
  p.outlierRejectionPercentile = 0.0f;  // clamped, but "pct > 0.0f" is false: no trimming.

  EXPECT_NO_THROW(recolorize3Dpc(pnts, map.get(), p));
}

TEST(CustomizableObsViz, RecolorizePinnedRangeSkipsOutlierRejection)
{
  auto map = buildGenericMapForColoring(5);
  const std::string field = "test_field_pinned";
  map->registerField_float(field);
  auto* buf = map->getPointsBufferRef_float_field(field);
  ASSERT_NE(buf, nullptr);
  for (size_t i = 0; i < buf->size(); i++) (*buf)[i] = static_cast<float>(i);

  auto pnts = mrpt::viz::CPointCloudColoured::Create();
  pnts->loadFromPointsMap(map.get());

  PointCloudRecoloringParameters p;
  p.colorizeByField = field;
  p.outlierRejectionPercentile = 0.1f;
  p.colorMapMinCoord = 0.0f;
  p.colorMapMaxCoord = 4.0f;

  EXPECT_NO_THROW(recolorize3Dpc(pnts, map.get(), p));
}

// =========================================================================
//  obs*_to_viz() free functions
// =========================================================================

TEST(CustomizableObsViz, Obs3DScanToViz)
{
  auto obs = CObservation3DRangeScan::Create();
  obs->hasPoints3D = true;
  obs->points3D_x = {1.0f, 2.0f, 3.0f, 4.0f};
  obs->points3D_y = {0.0f, 1.0f, 0.0f, 1.0f};
  obs->points3D_z = {0.0f, 0.0f, 1.0f, 1.0f};

  VisualizationParameters p;
  mrpt::viz::CSetOfObjects out;
  obs3Dscan_to_viz(obs, p, out);

  ASSERT_FALSE(out.empty());
  auto pnts = out.getByClass<mrpt::viz::CPointCloudColoured>();
  ASSERT_TRUE(pnts);
  EXPECT_EQ(pnts->size(), 4u);
}

TEST(CustomizableObsViz, ObsVelodyneToViz)
{
  auto obs = CObservationVelodyneScan::Create();
  obs->point_cloud.x = {1.0f, 2.0f, 3.0f};
  obs->point_cloud.y = {0.0f, 1.0f, 0.0f};
  obs->point_cloud.z = {0.0f, 0.0f, 1.0f};
  obs->point_cloud.intensity = {10, 20, 30};

  VisualizationParameters p;
  p.colorFromRGBimage = false;
  p.coloring.colorizeByField = "intensity";
  mrpt::viz::CSetOfObjects out;
  obsVelodyne_to_viz(obs, p, out);

  ASSERT_FALSE(out.empty());
  auto pnts = out.getByClass<mrpt::viz::CPointCloudColoured>();
  ASSERT_TRUE(pnts);
  EXPECT_EQ(pnts->size(), 3u);
}

TEST(CustomizableObsViz, ObsPointCloudToViz)
{
  auto obs = CObservationPointCloud::Create();
  obs->pointcloud = mrpt::maps::CSimplePointsMap::Create();
  obs->pointcloud->insertPoint(1.0f, 1.0f, 1.0f);
  obs->pointcloud->insertPoint(2.0f, 2.0f, 2.0f);
  obs->sensorPose = mrpt::poses::CPose3D(1, 0, 0, 0, 0, 0);

  VisualizationParameters p;
  mrpt::viz::CSetOfObjects out;
  obsPointCloud_to_viz(obs, p, out);

  ASSERT_FALSE(out.empty());
  auto pnts = out.getByClass<mrpt::viz::CPointCloudColoured>();
  ASSERT_TRUE(pnts);
  EXPECT_EQ(pnts->size(), 2u);
}

TEST(CustomizableObsViz, ObsPointCloudToVizNullCloud)
{
  auto obs = CObservationPointCloud::Create();
  // obs->pointcloud left as nullptr.

  VisualizationParameters p;
  mrpt::viz::CSetOfObjects out;
  EXPECT_NO_THROW(obsPointCloud_to_viz(obs, p, out));

  auto pnts = out.getByClass<mrpt::viz::CPointCloudColoured>();
  ASSERT_TRUE(pnts);
  EXPECT_EQ(pnts->size(), 0u);
}

TEST(CustomizableObsViz, ObsRotatingScanToViz)
{
  auto obs = CObservationRotatingScan::Create();
  obs->rowCount = 1;
  obs->columnCount = 3;
  obs->organizedPoints.resize(1, 3);
  obs->organizedPoints(0, 0) = mrpt::math::TPoint3Df(1, 0, 0);
  obs->organizedPoints(0, 1) = mrpt::math::TPoint3Df(0, 1, 0);
  obs->organizedPoints(0, 2) = mrpt::math::TPoint3Df(0, 0, 1);

  VisualizationParameters p;
  mrpt::viz::CSetOfObjects out;
  obsRotatingScan_to_viz(obs, p, out);

  ASSERT_FALSE(out.empty());
  auto pnts = out.getByClass<mrpt::viz::CPointCloudColoured>();
  ASSERT_TRUE(pnts);
  EXPECT_EQ(pnts->size(), 3u);
}

TEST(CustomizableObsViz, Obs2DScanToViz)
{
  auto obs = CObservation2DRangeScan::Create();
  obs->resizeScanAndAssign(10, 3.0f, true);
  obs->aperture = static_cast<float>(M_PI);

  VisualizationParameters p;
  p.showSurfaceIn2Dscans = true;
  p.showPointsIn2Dscans = true;
  mrpt::viz::CSetOfObjects out;
  obs2Dscan_to_viz(obs, p, out);

  ASSERT_FALSE(out.empty());
  auto laserScan = out.getByClass<mrpt::viz::CPlanarLaserScan>();
  ASSERT_TRUE(laserScan);
  laserScan->updateBuffers();
  EXPECT_GT(laserScan->shaderPointsVertexPointBuffer().size(), 0u);
}

TEST(CustomizableObsViz, AddCommonToVizTogglesAxisAndSensorPose)
{
  auto obs = CObservation2DRangeScan::Create();
  obs->resizeScanAndAssign(5, 2.0f, true);

  VisualizationParameters pFull;
  pFull.showAxis = true;
  pFull.drawSensorPose = true;
  mrpt::viz::CSetOfObjects outFull;
  obs2Dscan_to_viz(obs, pFull, outFull);

  VisualizationParameters pMinimal;
  pMinimal.showAxis = false;
  pMinimal.drawSensorPose = false;
  mrpt::viz::CSetOfObjects outMinimal;
  obs2Dscan_to_viz(obs, pMinimal, outMinimal);

  EXPECT_GT(outFull.size(), outMinimal.size());
}

TEST(CustomizableObsViz, ObsToVizDispatchesKnownTypes)
{
  VisualizationParameters p;

  {
    auto obs2D = CObservation2DRangeScan::Create();
    obs2D->resizeScanAndAssign(5, 2.0f, true);
    mrpt::viz::CSetOfObjects out;
    EXPECT_TRUE(obs_to_viz(mrpt::obs::CObservation::Ptr(obs2D), p, out));
    EXPECT_FALSE(out.empty());
  }
  {
    auto obs3D = CObservation3DRangeScan::Create();
    obs3D->hasPoints3D = true;
    obs3D->points3D_x = {1.0f};
    obs3D->points3D_y = {0.0f};
    obs3D->points3D_z = {0.0f};
    mrpt::viz::CSetOfObjects out;
    EXPECT_TRUE(obs_to_viz(mrpt::obs::CObservation::Ptr(obs3D), p, out));
    EXPECT_FALSE(out.empty());
  }
  {
    auto obsPC = CObservationPointCloud::Create();
    obsPC->pointcloud = mrpt::maps::CSimplePointsMap::Create();
    obsPC->pointcloud->insertPoint(1, 1, 1);
    mrpt::viz::CSetOfObjects out;
    EXPECT_TRUE(obs_to_viz(mrpt::obs::CObservation::Ptr(obsPC), p, out));
    EXPECT_FALSE(out.empty());
  }
}

TEST(CustomizableObsViz, ObsToVizUnknownTypeReturnsFalse)
{
  auto obsOdo = CObservationOdometry::Create();

  VisualizationParameters p;
  mrpt::viz::CSetOfObjects out;
  out.insert(mrpt::viz::CPointCloudColoured::Create());  // pre-populate to check it gets cleared

  const bool ok = obs_to_viz(mrpt::obs::CObservation::Ptr(obsOdo), p, out);
  EXPECT_FALSE(ok);
  EXPECT_TRUE(out.empty());
}

TEST(CustomizableObsViz, ObsToVizSensoryFrame)
{
  CSensoryFrame sf;

  auto obs2D = CObservation2DRangeScan::Create();
  obs2D->resizeScanAndAssign(5, 2.0f, true);
  sf.insert(obs2D);

  auto obs3D = CObservation3DRangeScan::Create();
  obs3D->hasPoints3D = true;
  obs3D->points3D_x = {1.0f};
  obs3D->points3D_y = {0.0f};
  obs3D->points3D_z = {0.0f};
  sf.insert(obs3D);

  VisualizationParameters p;
  mrpt::viz::CSetOfObjects out;
  const bool ok = obs_to_viz(sf, p, out);

  EXPECT_TRUE(ok);
  EXPECT_EQ(out.size(), 2u);
}

TEST(CustomizableObsViz, ObsToVizSensoryFrameEmptyReturnsFalse)
{
  CSensoryFrame sf;

  VisualizationParameters p;
  mrpt::viz::CSetOfObjects out;
  const bool ok = obs_to_viz(sf, p, out);

  EXPECT_FALSE(ok);
  EXPECT_TRUE(out.empty());
}
