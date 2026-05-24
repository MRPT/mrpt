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
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CReflectivityGridMap2D.h>
#include <mrpt/obs/CObservationReflectivity.h>
#include <mrpt/serialization/CArchive.h>

using mrpt::maps::CReflectivityGridMap2D;

// =========================================================================
//  Basic construction
// =========================================================================

TEST(CReflectivityGridMap2D, ConstructionAndGridSize)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.1);
  // Grid should have non-zero extent after construction.
  EXPECT_GT(m.getSizeX(), 0u);
  EXPECT_GT(m.getSizeY(), 0u);
  // Note: isEmpty() is a stub that always returns false for this map type.
}

TEST(CReflectivityGridMap2D, AsString)
{
  CReflectivityGridMap2D m;
  EXPECT_FALSE(m.asString().empty());
}

// =========================================================================
//  cell2float basic sanity
// =========================================================================

TEST(CReflectivityGridMap2D, Cell2Float)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.1);
  // The default log-odds cell value (0) maps to 0.5 probability.
  const int8_t neutral = 0;
  const float p = m.cell2float(neutral);
  EXPECT_NEAR(p, 0.5f, 0.05f);
}

// =========================================================================
//  Insert a CObservationReflectivity and verify the map is no longer empty
// =========================================================================

TEST(CReflectivityGridMap2D, InsertObservation)
{
  CReflectivityGridMap2D m(-5, 5, -5, 5, 0.1);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.8f;  // bright surface
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);

  mrpt::poses::CPose3D robotPose;
  m.insertObservation(obs, robotPose);

  EXPECT_FALSE(m.isEmpty());
}

// =========================================================================
//  clear resets cells to neutral probability
// =========================================================================

TEST(CReflectivityGridMap2D, ClearResetsToNeutral)
{
  CReflectivityGridMap2D m(-5, 5, -5, 5, 0.1);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.9f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  mrpt::poses::CPose3D robotPose;
  m.insertObservation(obs, robotPose);

  // After insertion at least the sensor cell should differ from neutral (0).
  // After clear, internal_clear fills with neutral log-odds => cell2float ~0.5.
  m.clear();

  // Spot-check the centre cell: after clear it should be near neutral (0.5).
  const auto* cell = m.cellByPos(0.0, 0.0);
  ASSERT_NE(cell, nullptr);
  EXPECT_NEAR(m.cell2float(*cell), 0.5f, 0.05f);
}

// =========================================================================
//  getAsImage smoke test
// =========================================================================

TEST(CReflectivityGridMap2D, GetAsImage)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.5);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.6f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  mrpt::poses::CPose3D robotPose;
  m.insertObservation(obs, robotPose);

  mrpt::img::CImage img;
  m.getAsImage(img);
  EXPECT_GT(img.getWidth(), 0u);
  EXPECT_GT(img.getHeight(), 0u);

  // Also with verticalFlip and forceRGB
  mrpt::img::CImage img2;
  m.getAsImage(img2, /*verticalFlip=*/true, /*forceRGB=*/true);
  EXPECT_EQ(img2.getWidth(), img.getWidth());
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(CReflectivityGridMap2D, SerializeRoundTrip)
{
  CReflectivityGridMap2D src(-3, 3, -3, 3, 0.2);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.9f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  mrpt::poses::CPose3D robotPose;
  src.insertObservation(obs, robotPose);

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  CReflectivityGridMap2D dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    mrpt::serialization::CSerializable::Ptr obj;
    ar >> obj;
    ASSERT_NE(obj, nullptr);
    dst = *dynamic_cast<CReflectivityGridMap2D*>(obj.get());
  }

  EXPECT_FALSE(dst.isEmpty());
}

// =========================================================================
//  saveMetricMapRepresentationToFile smoke test
// =========================================================================

TEST(CReflectivityGridMap2D, SaveRepresentation)
{
  CReflectivityGridMap2D m(-2, 2, -2, 2, 0.2);

  mrpt::obs::CObservationReflectivity obs;
  obs.reflectivityLevel = 0.7f;
  obs.sensorPose = mrpt::poses::CPose3D(0, 0, 0, 0, 0, 0);
  mrpt::poses::CPose3D robotPose;
  m.insertObservation(obs, robotPose);

  // Should not throw
  EXPECT_NO_THROW(m.saveMetricMapRepresentationToFile("/tmp/reflectivity_test"));
}
