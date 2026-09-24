/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Unit tests for converting point maps into viz::CPointCloudColoured, and for
// the color accessors of the CPointsMap point cloud adapter.

#include <gtest/gtest.h>
#include <mrpt/img/color_maps.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/viz/CPointCloudColoured.h>

using mrpt::maps::CGenericPointsMap;
using mrpt::maps::CPointsMap;

namespace
{
constexpr size_t N = 50;

CGenericPointsMap::Ptr makeMap(bool u8Colors, bool fColors)
{
  auto m = CGenericPointsMap::Create();
  if (u8Colors)
  {
    m->registerField_uint8(CPointsMap::POINT_FIELD_COLOR_Ru8);
    m->registerField_uint8(CPointsMap::POINT_FIELD_COLOR_Gu8);
    m->registerField_uint8(CPointsMap::POINT_FIELD_COLOR_Bu8);
  }
  if (fColors)
  {
    m->registerField_float(CPointsMap::POINT_FIELD_COLOR_Rf);
    m->registerField_float(CPointsMap::POINT_FIELD_COLOR_Gf);
    m->registerField_float(CPointsMap::POINT_FIELD_COLOR_Bf);
  }
  for (size_t i = 0; i < N; i++)
  {
    const auto f = static_cast<float>(i);
    m->insertPointFast(f, 2 * f, -f);
    if (u8Colors)
    {
      m->insertPointField_uint8(CPointsMap::POINT_FIELD_COLOR_Ru8, static_cast<uint8_t>(i));
      m->insertPointField_uint8(CPointsMap::POINT_FIELD_COLOR_Gu8, static_cast<uint8_t>(2 * i));
      m->insertPointField_uint8(CPointsMap::POINT_FIELD_COLOR_Bu8, static_cast<uint8_t>(3 * i));
    }
    if (fColors)
    {
      m->insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Rf, 0.01f * f);
      m->insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Gf, 0.5f);
      m->insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Bf, 1.0f);
    }
  }
  return m;
}
}  // namespace

TEST(CPointCloudColoured, loadFromPointsMapNoColor)
{
  const auto m = makeMap(false, false);
  auto gl = mrpt::viz::CPointCloudColoured::Create();
  gl->loadFromPointsMap(m.get());

  ASSERT_EQ(gl->size(), N);
  for (size_t i = 0; i < N; i++)
  {
    const auto& p = gl->getPoint3Df(i);
    EXPECT_FLOAT_EQ(p.x, static_cast<float>(i));
    EXPECT_FLOAT_EQ(p.y, 2.0f * static_cast<float>(i));
    EXPECT_FLOAT_EQ(p.z, -static_cast<float>(i));
    const auto c = gl->getPointColor(i);
    EXPECT_EQ(c.R, 0);
    EXPECT_EQ(c.G, 0);
    EXPECT_EQ(c.B, 0);
    EXPECT_EQ(c.A, 0xff);
  }
}

TEST(CPointCloudColoured, loadFromPointsMapColorU8)
{
  const auto m = makeMap(true, false);
  auto gl = mrpt::viz::CPointCloudColoured::Create();
  gl->loadFromPointsMap(m.get());

  ASSERT_EQ(gl->size(), N);
  for (size_t i = 0; i < N; i++)
  {
    const auto c = gl->getPointColor(i);
    EXPECT_EQ(c.R, i);
    EXPECT_EQ(c.G, 2 * i);
    EXPECT_EQ(c.B, 3 * i);
  }
}

TEST(CPointCloudColoured, loadFromPointsMapColorFloat)
{
  const auto m = makeMap(false, true);
  auto gl = mrpt::viz::CPointCloudColoured::Create();
  gl->loadFromPointsMap(m.get());

  ASSERT_EQ(gl->size(), N);
  for (size_t i = 0; i < N; i++)
  {
    const auto c = gl->getPointColor(i);
    EXPECT_EQ(c.R, mrpt::f2u8(0.01f * static_cast<float>(i)));
    EXPECT_EQ(c.G, mrpt::f2u8(0.5f));
    EXPECT_EQ(c.B, 0xff);
  }
}

TEST(CPointCloudColoured, recolorizeByCoordinate)
{
  const auto m = makeMap(false, false);
  auto gl = mrpt::viz::CPointCloudColoured::Create();
  gl->loadFromPointsMap(m.get());
  // x spans [0, N-1]:
  gl->recolorizeByCoordinate(0.0f, static_cast<float>(N - 1), 0 /*x*/, mrpt::img::cmJET);

  const auto expectColor = [](float v)
  {
    const auto c = mrpt::img::colormap(mrpt::img::cmJET, v);
    return mrpt::img::TColor(mrpt::f2u8(c.R), mrpt::f2u8(c.G), mrpt::f2u8(c.B));
  };
  for (size_t i : {size_t(0), N / 2, N - 1})
  {
    const auto c = gl->getPointColor(i);
    const auto ref = expectColor(static_cast<float>(i) / static_cast<float>(N - 1));
    const float tol = 3;  // colormap quantization, in u8 levels
    EXPECT_NEAR(c.R, ref.R, tol) << "i=" << i;
    EXPECT_NEAR(c.G, ref.G, tol) << "i=" << i;
    EXPECT_NEAR(c.B, ref.B, tol) << "i=" << i;
    EXPECT_EQ(c.A, 0xff);
  }
}

TEST(CPointsMapAdapter, setPointRGBu8WritesU8Fields)
{
  auto m = makeMap(true, false);
  mrpt::viz::PointCloudAdapter<CPointsMap> adapter(*m);
  adapter.setPointRGBu8(3, 10, 20, 30);
  EXPECT_EQ(m->getPointField_uint8(3, CPointsMap::POINT_FIELD_COLOR_Ru8), 10);
  EXPECT_EQ(m->getPointField_uint8(3, CPointsMap::POINT_FIELD_COLOR_Gu8), 20);
  EXPECT_EQ(m->getPointField_uint8(3, CPointsMap::POINT_FIELD_COLOR_Bu8), 30);

  adapter.setPointRGBf(4, 1.0f, 0.0f, 1.0f);
  EXPECT_EQ(m->getPointField_uint8(4, CPointsMap::POINT_FIELD_COLOR_Ru8), 0xff);
  EXPECT_EQ(m->getPointField_uint8(4, CPointsMap::POINT_FIELD_COLOR_Gu8), 0);
  EXPECT_EQ(m->getPointField_uint8(4, CPointsMap::POINT_FIELD_COLOR_Bu8), 0xff);
}
