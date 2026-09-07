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
#include <mrpt/math/CPolygon.h>
#include <mrpt/serialization/CArchive.h>

#include "legacy_serialization.h"

using namespace mrpt::math;

TEST(CPolygon, SetGetVerticesDouble)
{
  CPolygon p;
  const std::vector<double> xs{0, 1, 1, 0};
  const std::vector<double> ys{0, 0, 1, 1};
  p.set_vertices(xs, ys);

  std::vector<double> ox;
  std::vector<double> oy;
  p.get_vertices(ox, oy);
  ASSERT_EQ(ox.size(), 4u);
  EXPECT_NEAR(ox[2], 1.0, 1e-9);
  EXPECT_NEAR(oy[2], 1.0, 1e-9);
}

TEST(CPolygon, SetVerticesFloatPointers)
{
  CPolygon p;
  const float xs[3] = {0.0f, 1.0f, 0.5f};
  const float ys[3] = {0.0f, 0.0f, 1.0f};
  p.set_vertices(3, xs, ys);

  EXPECT_EQ(p.size(), 3u);
  EXPECT_NEAR(p[1].x, 1.0, 1e-6);
  EXPECT_NEAR(p[2].y, 1.0, 1e-6);
}

TEST(CPolygon, SetVerticesMismatchedSizesThrows)
{
  CPolygon p;
  const std::vector<double> xs{0, 1};
  const std::vector<double> ys{0};
  EXPECT_THROW(p.set_vertices(xs, ys), std::exception);
}

TEST(CPolygon, SerializationRoundTripCurrentVersion)
{
  CPolygon p;
  p.set_vertices(std::vector<double>{0, 1, 1, 0}, std::vector<double>{0, 0, 1, 1});

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << p;
  membuf.Seek(0);

  CPolygon p2;
  arch >> p2;

  ASSERT_EQ(p2.size(), p.size());
  for (size_t i = 0; i < p.size(); i++)
  {
    EXPECT_NEAR(p[i].x, p2[i].x, 1e-9);
    EXPECT_NEAR(p[i].y, p2[i].y, 1e-9);
  }
}

TEST(CPolygon, SerializationRoundTripEmpty)
{
  CPolygon p;
  p.clear();

  mrpt::io::CMemoryStream membuf;
  auto arch = mrpt::serialization::archiveFrom(membuf);
  arch << p;
  membuf.Seek(0);

  CPolygon p2;
  arch >> p2;
  EXPECT_EQ(p2.size(), 0u);
}

// Writes the payload of the pre-v2 (float/double, explicit bounding box)
// polygon format, shared by streaming versions 0 and 1.
namespace
{
void writeLegacyPolygonPayload(
    mrpt::serialization::CArchive& arch,
    const std::vector<double>& xs,
    const std::vector<double>& ys)
{
  const auto n = static_cast<uint32_t>(xs.size());
  arch << n;
  // max_x, max_y, min_x, min_y, cx, cy: read but discarded by the loader
  for (int i = 0; i < 6; i++) arch << double(0);
  for (const auto v : xs) arch << v;
  for (const auto v : ys) arch << v;
}
}  // namespace

TEST(CPolygon, DeserializeLegacyVersion0)
{
  const std::vector<double> xs{0, 1, 1, 0};
  const std::vector<double> ys{0, 0, 1, 1};

  mrpt::io::CMemoryStream buf;
  mrpt_test::writeLegacyObjectFrame(
      buf, "CPolygon", 0,
      [&](mrpt::serialization::CArchive& a) { writeLegacyPolygonPayload(a, xs, ys); });

  auto arch = mrpt::serialization::archiveFrom(buf);
  const auto obj = arch.ReadObject();
  const auto p = std::dynamic_pointer_cast<CPolygon>(obj);
  ASSERT_TRUE(p);
  ASSERT_EQ(p->size(), 4U);
  EXPECT_NEAR(p->get_vertex_x(2), 1.0, 1e-9);
  EXPECT_NEAR(p->get_vertex_y(3), 1.0, 1e-9);
}

TEST(CPolygon, DeserializeLegacyVersion1)
{
  const std::vector<double> xs{0, 2, 2};
  const std::vector<double> ys{0, 0, 3};

  mrpt::io::CMemoryStream buf;
  mrpt_test::writeLegacyObjectFrame(
      buf, "CPolygon", 1,
      [&](mrpt::serialization::CArchive& a) { writeLegacyPolygonPayload(a, xs, ys); });

  auto arch = mrpt::serialization::archiveFrom(buf);
  const auto obj = arch.ReadObject();
  const auto p = std::dynamic_pointer_cast<CPolygon>(obj);
  ASSERT_TRUE(p);
  ASSERT_EQ(p->size(), 3U);
  EXPECT_NEAR(p->get_vertex_x(1), 2.0, 1e-9);
  EXPECT_NEAR(p->get_vertex_y(2), 3.0, 1e-9);
}

TEST(CPolygon, DeserializeUnknownVersionThrows)
{
  mrpt::io::CMemoryStream buf;
  mrpt_test::writeLegacyObjectFrame(buf, "CPolygon", 99, [](mrpt::serialization::CArchive&) {});

  auto arch = mrpt::serialization::archiveFrom(buf);
  EXPECT_THROW(arch.ReadObject(), std::exception);
}

TEST(CPolygon, VertexAccessorsCheckBounds)
{
  CPolygon p;
  p.add_vertex(1.0, 2.0);
  EXPECT_NEAR(p.get_vertex_x(0), 1.0, 1e-9);
  EXPECT_NEAR(p.get_vertex_y(0), 2.0, 1e-9);
  EXPECT_THROW(p.get_vertex_x(1), std::exception);
  EXPECT_THROW(p.get_vertex_y(1), std::exception);
}
