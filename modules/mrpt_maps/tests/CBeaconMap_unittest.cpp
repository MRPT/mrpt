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
#include <mrpt/maps/CBeacon.h>
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/serialization/CArchive.h>

#include <filesystem>

using mrpt::maps::CBeacon;
using mrpt::maps::CBeaconMap;

static CBeacon makeGaussianBeacon(mrpt::maps::CBeacon::TBeaconID id, double x, double y, double z)
{
  CBeacon b;
  b.m_ID = id;
  b.m_typePDF = CBeacon::pdfGauss;
  b.m_locationGauss.mean = mrpt::poses::CPoint3D(x, y, z);
  b.m_locationGauss.cov.setIdentity();
  b.m_locationGauss.cov *= 0.01;
  return b;
}

// =========================================================================
//  Basic construction and isEmpty
// =========================================================================

TEST(CBeaconMap, EmptyOnConstruction)
{
  CBeaconMap m;
  EXPECT_TRUE(m.isEmpty());
  EXPECT_EQ(m.size(), 0u);
}

// =========================================================================
//  push_back, size, operator[], get
// =========================================================================

TEST(CBeaconMap, PushBackAndAccess)
{
  CBeaconMap m;

  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 0.0));
  m.push_back(makeGaussianBeacon(2, 3.0, 4.0, 0.0));
  m.push_back(makeGaussianBeacon(3, 5.0, 6.0, 0.0));

  EXPECT_FALSE(m.isEmpty());
  EXPECT_EQ(m.size(), 3u);

  EXPECT_EQ(m[0].m_ID, 1);
  EXPECT_EQ(m[1].m_ID, 2);
  EXPECT_EQ(m.get(2).m_ID, 3);

  EXPECT_NEAR(m[0].m_locationGauss.mean.x(), 1.0, 1e-9);
  EXPECT_NEAR(m[1].m_locationGauss.mean.y(), 4.0, 1e-9);
}

// =========================================================================
//  Mutable access via operator[] and get
// =========================================================================

TEST(CBeaconMap, MutableAccess)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(10, 0, 0, 0));

  m[0].m_ID = 99;
  EXPECT_EQ(m.get(0).m_ID, 99);
}

// =========================================================================
//  Iterators
// =========================================================================

TEST(CBeaconMap, IteratorAccess)
{
  CBeaconMap m;
  for (int i = 0; i < 5; ++i) m.push_back(makeGaussianBeacon(i, double(i), 0, 0));

  int count = 0;
  for (const auto& b : m)
  {
    EXPECT_EQ(b.m_ID, count);
    ++count;
  }
  EXPECT_EQ(count, 5);
}

// =========================================================================
//  internal_clear via the CMetricMap interface
// =========================================================================

TEST(CBeaconMap, Clear)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1, 1, 1));
  m.push_back(makeGaussianBeacon(2, 2, 2, 2));
  EXPECT_EQ(m.size(), 2u);

  m.clear();
  EXPECT_TRUE(m.isEmpty());
  EXPECT_EQ(m.size(), 0u);
}

// =========================================================================
//  resize
// =========================================================================

TEST(CBeaconMap, Resize)
{
  CBeaconMap m;
  m.resize(4);
  EXPECT_EQ(m.size(), 4u);

  m.resize(2);
  EXPECT_EQ(m.size(), 2u);
}

// =========================================================================
//  Serialization round-trip
// =========================================================================

TEST(CBeaconMap, SerializeRoundTrip)
{
  CBeaconMap src;
  src.push_back(makeGaussianBeacon(7, 1.5, 2.5, 0.5));
  src.push_back(makeGaussianBeacon(8, -1.0, -2.0, 0.0));

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  CBeaconMap dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    mrpt::serialization::CSerializable::Ptr obj;
    ar >> obj;
    ASSERT_NE(obj, nullptr);
    auto* ptr = dynamic_cast<CBeaconMap*>(obj.get());
    ASSERT_NE(ptr, nullptr);
    dst = *ptr;
  }

  EXPECT_EQ(dst.size(), src.size());
  EXPECT_EQ(dst[0].m_ID, 7);
  EXPECT_EQ(dst[1].m_ID, 8);
  EXPECT_NEAR(dst[0].m_locationGauss.mean.x(), 1.5, 1e-6);
  EXPECT_NEAR(dst[1].m_locationGauss.mean.y(), -2.0, 1e-6);
}

// =========================================================================
//  changeCoordinatesReference
// =========================================================================

TEST(CBeaconMap, ChangeCoordinatesReference)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 0.0, 0.0));

  // Translate by (10, 0, 0)
  mrpt::poses::CPose3D offset(10.0, 0.0, 0.0, 0, 0, 0);
  m.changeCoordinatesReference(offset);

  EXPECT_NEAR(m[0].m_locationGauss.mean.x(), 11.0, 1e-6);
}

// =========================================================================
//  saveToMATLABScript3D (just check it returns true without crashing)
// =========================================================================

TEST(CBeaconMap, SaveToMATLABScript)
{
  CBeaconMap m;
  m.push_back(makeGaussianBeacon(1, 1.0, 2.0, 0.0));

  const auto fname = (std::filesystem::temp_directory_path() / "test_beaconmap.m").string();
  bool ok = m.saveToMATLABScript3D(fname);
  EXPECT_TRUE(ok);
}
