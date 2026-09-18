/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Unit tests for the deserialization-only CPointsMapXYZIRT compat stub.

#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CPointsMapXYZIRT.h>
#include <mrpt/rtti/CObject.h>
#include <mrpt/serialization/CArchive.h>

// The class under test is deprecated on purpose; testing it is the point.
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
#if defined(_MSC_VER)
#pragma warning(disable : 4996)
#endif

using mrpt::maps::CGenericPointsMap;
using mrpt::maps::CPointsMap;
using mrpt::maps::CPointsMapXYZIRT;

namespace
{
CPointsMapXYZIRT makeTestMap()
{
  CPointsMapXYZIRT m;
  m.registerField_float(CPointsMap::POINT_FIELD_INTENSITY);
  m.registerField_uint16(CPointsMap::POINT_FIELD_RING_ID);
  m.registerField_float(CPointsMap::POINT_FIELD_TIMESTAMP);

  const size_t N = 4;
  m.resize(N);
  for (size_t i = 0; i < N; i++)
  {
    m.setPointFast(i, float(i), float(i * 10), float(i * 100));
    m.setPointField_float(i, CPointsMap::POINT_FIELD_INTENSITY, float(i) + 0.25f);
    m.setPointField_uint16(i, CPointsMap::POINT_FIELD_RING_ID, static_cast<uint16_t>(i + 1));
    m.setPointField_float(i, CPointsMap::POINT_FIELD_TIMESTAMP, float(i) * 0.01f);
  }
  return m;
}
}  // namespace

// Without the class staying registered under its original name, reading any
// archive holding one of these throws "class ... is not registered".
TEST(CPointsMapXYZIRT, RegisteredUnderLegacyClassName)
{
  const auto* cls = mrpt::rtti::findRegisteredClass("mrpt::maps::CPointsMapXYZIRT", false);
  ASSERT_NE(cls, nullptr);

  const auto obj = cls->createObject();
  ASSERT_NE(obj, nullptr);
  EXPECT_NE(std::dynamic_pointer_cast<CGenericPointsMap>(obj), nullptr);
}

TEST(CPointsMapXYZIRT, SerializationRoundTrip)
{
  mrpt::io::CMemoryStream buf;
  {
    const auto src = makeTestMap();
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }

  buf.Seek(0);
  auto ar = mrpt::serialization::archiveFrom(buf);
  mrpt::serialization::CSerializable::Ptr obj;
  ar >> obj;

  const auto m = std::dynamic_pointer_cast<CGenericPointsMap>(obj);
  ASSERT_NE(m, nullptr);
  ASSERT_EQ(m->size(), 4U);

  EXPECT_TRUE(m->hasPointField(CPointsMap::POINT_FIELD_INTENSITY));
  EXPECT_TRUE(m->hasPointField(CPointsMap::POINT_FIELD_RING_ID));
  EXPECT_TRUE(m->hasPointField(CPointsMap::POINT_FIELD_TIMESTAMP));

  for (size_t i = 0; i < m->size(); i++)
  {
    float x = 0;
    float y = 0;
    float z = 0;
    m->getPointFast(i, x, y, z);
    EXPECT_FLOAT_EQ(x, float(i));
    EXPECT_FLOAT_EQ(y, float(i * 10));
    EXPECT_FLOAT_EQ(z, float(i * 100));

    EXPECT_FLOAT_EQ(m->getPointField_float(i, CPointsMap::POINT_FIELD_INTENSITY), float(i) + 0.25f);
    EXPECT_EQ(
        m->getPointField_uint16(i, CPointsMap::POINT_FIELD_RING_ID), static_cast<uint16_t>(i + 1));
    EXPECT_FLOAT_EQ(m->getPointField_float(i, CPointsMap::POINT_FIELD_TIMESTAMP), float(i) * 0.01f);
  }
}

// A map with no optional fields must still round-trip (all counts zero).
TEST(CPointsMapXYZIRT, SerializationRoundTripWithoutOptionalFields)
{
  mrpt::io::CMemoryStream buf;
  {
    CPointsMapXYZIRT src;
    src.resize(2);
    src.setPointFast(0, 1.0f, 2.0f, 3.0f);
    src.setPointFast(1, 4.0f, 5.0f, 6.0f);

    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }

  buf.Seek(0);
  auto ar = mrpt::serialization::archiveFrom(buf);
  mrpt::serialization::CSerializable::Ptr obj;
  ar >> obj;

  const auto m = std::dynamic_pointer_cast<CGenericPointsMap>(obj);
  ASSERT_NE(m, nullptr);
  ASSERT_EQ(m->size(), 2U);
  EXPECT_FALSE(m->hasPointField(CPointsMap::POINT_FIELD_INTENSITY));

  float x = 0;
  float y = 0;
  float z = 0;
  m->getPointFast(1, x, y, z);
  EXPECT_FLOAT_EQ(x, 4.0f);
  EXPECT_FLOAT_EQ(y, 5.0f);
  EXPECT_FLOAT_EQ(z, 6.0f);
}
