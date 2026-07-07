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
