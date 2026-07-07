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
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/serialization/CArchive.h>

TEST(CPoint, toFromString_2D)
{
  mrpt::poses::CPoint2D p;

  p.fromString("[1.0 2.0]");
  EXPECT_DOUBLE_EQ(p.x(), 1.0);
  EXPECT_DOUBLE_EQ(p.y(), 2.0);

  const auto s = p.asString();
  EXPECT_EQ(s, "[1.000000 2.000000]");

  EXPECT_THROW(p.fromString("[]"), std::exception);
  EXPECT_THROW(p.fromString("[1 2;4 5]"), std::exception);
  EXPECT_THROW(p.fromString("[1 2 3]"), std::exception);
  EXPECT_THROW(p.fromString("[1;2]"), std::exception);
}

TEST(CPoint, toFromString_3D)
{
  mrpt::poses::CPoint3D p;

  p.fromString("[1.0 2.0 3.0]");
  EXPECT_DOUBLE_EQ(p.x(), 1.0);
  EXPECT_DOUBLE_EQ(p.y(), 2.0);
  EXPECT_DOUBLE_EQ(p.z(), 3.0);

  const auto s = p.asString();
  EXPECT_EQ(s, "[1.000000 2.000000 3.000000]");

  EXPECT_THROW(p.fromString("[]"), std::exception);
  EXPECT_THROW(p.fromString("[1 2;4 5]"), std::exception);
  EXPECT_THROW(p.fromString("[1 2]"), std::exception);
  EXPECT_THROW(p.fromString("[1;2;3]"), std::exception);
}

TEST(CPoint, CPoint2D_ConstructorsOperatorsAndSerialization)
{
  using namespace mrpt::poses;
  using namespace mrpt::math;

  CPoint2D p(1.0, 2.0);
  EXPECT_DOUBLE_EQ(p.x(), 1.0);
  EXPECT_DOUBLE_EQ(p.y(), 2.0);

  CPoint2D pFromT(TPoint2D(3.0, 4.0));
  EXPECT_DOUBLE_EQ(pFromT.x(), 3.0);
  CPoint2D pFromT3(TPoint3D(5.0, 6.0, 7.0));
  EXPECT_DOUBLE_EQ(pFromT3.x(), 5.0);
  EXPECT_DOUBLE_EQ(pFromT3.y(), 6.0);

  TPoint2D tp = p.asTPoint();
  EXPECT_DOUBLE_EQ(tp.x, 1.0);
  EXPECT_DOUBLE_EQ(tp.y, 2.0);

  CPose2D pose(0, 0, 0);
  CPoint2D diff = p - pose;
  EXPECT_NEAR(diff.x(), 1.0, 1e-9);
  EXPECT_NEAR(diff.y(), 2.0, 1e-9);

  std::ostringstream ss;
  ss << p;
  EXPECT_FALSE(ss.str().empty());

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p;
  buf.Seek(0);
  CPoint2D p2;
  arch >> p2;
  EXPECT_DOUBLE_EQ(p2.x(), p.x());
  EXPECT_DOUBLE_EQ(p2.y(), p.y());
}

TEST(CPoint, CPoint3D_ConstructorsOperatorsAndSerialization)
{
  using namespace mrpt::poses;
  using namespace mrpt::math;

  CPoint3D p(1.0, 2.0, 3.0);
  CPoint2D p2d(1.0, 2.0);
  CPoint3D pFrom2D(p2d);
  EXPECT_DOUBLE_EQ(pFrom2D.x(), 1.0);
  EXPECT_DOUBLE_EQ(pFrom2D.z(), 0.0);

  CPose2D pose2d(1, 2, 0.1);
  CPoint3D pFromPose2D(pose2d);
  EXPECT_DOUBLE_EQ(pFromPose2D.x(), 1.0);
  EXPECT_DOUBLE_EQ(pFromPose2D.z(), 0.0);

  CPose3D pose3d(1, 2, 3, 0.1, 0.2, 0.3);
  CPoint3D pFromPose3D(pose3d);
  EXPECT_DOUBLE_EQ(pFromPose3D.x(), 1.0);
  EXPECT_DOUBLE_EQ(pFromPose3D.z(), 3.0);

  CPoint3D diffPose = p - CPose3D(0, 0, 0, 0, 0, 0);
  EXPECT_NEAR(diffPose.x(), 1.0, 1e-9);

  CPoint3D diffPoint = p - CPoint3D(1, 1, 1);
  EXPECT_NEAR(diffPoint.x(), 0.0, 1e-9);
  EXPECT_NEAR(diffPoint.y(), 1.0, 1e-9);

  CPoint3D sumPoint = p + CPoint3D(1, 1, 1);
  EXPECT_NEAR(sumPoint.x(), 2.0, 1e-9);

  CPose3D sumPose = p + CPose3D(0, 0, 0, 0, 0, 0);
  EXPECT_NEAR(sumPose.x(), 1.0, 1e-9);

  TPoint3D tp = p.asTPoint();
  EXPECT_DOUBLE_EQ(tp.x, 1.0);

  std::ostringstream ss;
  ss << p;
  EXPECT_FALSE(ss.str().empty());

  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << p;
  buf.Seek(0);
  CPoint3D p3;
  arch >> p3;
  EXPECT_DOUBLE_EQ(p3.x(), p.x());
  EXPECT_DOUBLE_EQ(p3.z(), p.z());
}
