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
#include <mrpt/obs/stock_observations.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CPlanarLaserScan.h>

using mrpt::viz::CPlanarLaserScan;

TEST(CPlanarLaserScan, ConstructAndClear)
{
  CPlanarLaserScan obj;
  mrpt::obs::CObservation2DRangeScan scan;
  mrpt::obs::stock_observations::example2DRangeScan(scan);
  obj.setScan(scan);

  EXPECT_NO_THROW(obj.clear());
}

TEST(CPlanarLaserScan, BoundingBoxAllChannelsEnabled)
{
  CPlanarLaserScan obj;
  mrpt::obs::CObservation2DRangeScan scan;
  mrpt::obs::stock_observations::example2DRangeScan(scan);
  obj.setScan(scan);

  obj.enablePoints(true);
  obj.enableLine(true);
  obj.enableSurface(true);

  const auto bb = obj.getBoundingBox();
  EXPECT_GE(bb.max.x, bb.min.x);
}

TEST(CPlanarLaserScan, BoundingBoxAllChannelsDisabled)
{
  CPlanarLaserScan obj;
  mrpt::obs::CObservation2DRangeScan scan;
  mrpt::obs::stock_observations::example2DRangeScan(scan);
  obj.setScan(scan);

  obj.enablePoints(false);
  obj.enableLine(false);
  obj.enableSurface(false);

  EXPECT_NO_THROW(obj.getBoundingBox());
}

TEST(CPlanarLaserScan, GetLocalRepresentativePointMatchesSensorPose)
{
  CPlanarLaserScan obj;
  mrpt::obs::CObservation2DRangeScan scan;
  mrpt::obs::stock_observations::example2DRangeScan(scan);
  scan.sensorPose = mrpt::poses::CPose3D(1.0, 2.0, 3.0, 0, 0, 0);
  obj.setScan(scan);

  const auto pt = obj.getLocalRepresentativePoint();
  EXPECT_NEAR(pt.x, 1.0f, 1e-4f);
  EXPECT_NEAR(pt.y, 2.0f, 1e-4f);
  EXPECT_NEAR(pt.z, 3.0f, 1e-4f);
}

TEST(CPlanarLaserScan, ColorSetters)
{
  CPlanarLaserScan obj;
  EXPECT_NO_THROW(obj.setLineColor(0.1f, 0.2f, 0.3f, 0.4f));
  EXPECT_NO_THROW(obj.setPointsColor(0.5f, 0.6f, 0.7f, 0.8f));
  EXPECT_NO_THROW(obj.setSurfaceColor(0.9f, 0.1f, 0.2f, 0.3f));
}

TEST(CPlanarLaserScan, SerializeRoundTrip)
{
  CPlanarLaserScan src;
  mrpt::obs::CObservation2DRangeScan scan;
  mrpt::obs::stock_observations::example2DRangeScan(scan);
  src.setScan(scan);
  src.enablePoints(false);
  src.setLineColor(0.2f, 0.3f, 0.4f, 0.5f);

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  mrpt::serialization::CSerializable::Ptr obj;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar >> obj;
  }
  auto* dst = dynamic_cast<CPlanarLaserScan*>(obj.get());
  ASSERT_NE(dst, nullptr);

  // The bounding box should be computable post-deserialization without
  // throwing, exercising updateBuffers()'s lazy-cache fill on the restored
  // object:
  EXPECT_NO_THROW(dst->getBoundingBox());
}
