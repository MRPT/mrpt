/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Unit tests for the serialization of point maps with Cloudini compression.

#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

using mrpt::maps::CGenericPointsMap;
using mrpt::maps::CPointsMap;
using mrpt::maps::CSimplePointsMap;

namespace
{
// Sets the serialization options for the scope of a test.
class ScopedOptions
{
 public:
  explicit ScopedOptions(const CPointsMap::TSerializationOptions& o) :
      m_saved(CPointsMap::serializationOptions())
  {
    CPointsMap::serializationOptions() = o;
  }
  ~ScopedOptions() { CPointsMap::serializationOptions() = m_saved; }

 private:
  CPointsMap::TSerializationOptions m_saved;
};

std::vector<uint8_t> serialize(const mrpt::serialization::CSerializable& obj)
{
  mrpt::io::CMemoryStream buf;
  auto ar = mrpt::serialization::archiveFrom(buf);
  ar << obj;
  std::vector<uint8_t> data(buf.getTotalBytesCount());
  std::memcpy(data.data(), buf.getRawBufferData(), data.size());
  return data;
}

template <class MAP>
void deserialize(const std::vector<uint8_t>& data, MAP& obj)
{
  mrpt::io::CMemoryStream buf;
  buf.assignMemoryNotOwn(data.data(), data.size());
  auto ar = mrpt::serialization::archiveFrom(buf);
  ar >> obj;
}

// Points in a 200 m wide box, with some NaN coordinates:
void fillRandomXYZ(CPointsMap& m, size_t n)
{
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  m.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    m.setPointFast(
        i, rng.drawUniform<float>(-100, 100), rng.drawUniform<float>(-100, 100),
        rng.drawUniform<float>(-5, 30));
  }
  if (n > 10)
  {
    m.setPointFast(7, std::numeric_limits<float>::quiet_NaN(), 0, 0);
  }
}

void expectXYZNear(const CPointsMap& a, const CPointsMap& b, float maxErr)
{
  ASSERT_EQ(a.size(), b.size());
  const auto& ax = a.getPointsBufferRef_x();
  const auto& ay = a.getPointsBufferRef_y();
  const auto& az = a.getPointsBufferRef_z();
  const auto& bx = b.getPointsBufferRef_x();
  const auto& by = b.getPointsBufferRef_y();
  const auto& bz = b.getPointsBufferRef_z();
  for (size_t i = 0; i < a.size(); i++)
  {
    if (std::isnan(ax[i]))
    {
      EXPECT_TRUE(std::isnan(bx[i])) << "i=" << i;
      continue;
    }
    ASSERT_NEAR(ax[i], bx[i], maxErr) << "i=" << i;
    ASSERT_NEAR(ay[i], by[i], maxErr) << "i=" << i;
    ASSERT_NEAR(az[i], bz[i], maxErr) << "i=" << i;
  }
}

CGenericPointsMap makeGenericMap(size_t n)
{
  CGenericPointsMap m;
  m.registerField_float("intensity");
  m.registerField_float("t");
  m.registerField_double("gps_time");
  m.registerField_uint16("ring");
  m.registerField_uint8("color_r");
  m.registerField_uint32("id");
  fillRandomXYZ(m, n);
  auto& rng = mrpt::random::getRandomGenerator();
  for (size_t i = 0; i < n; i++)
  {
    m.setPointField_float(i, "intensity", rng.drawUniform<float>(0, 1));
    m.setPointField_float(i, "t", rng.drawUniform<float>(-0.05f, 0.05f));
    m.setPointField_double(i, "gps_time", 1.7e9 + 1e-3 * static_cast<double>(i));
    m.setPointField_uint16(i, "ring", static_cast<uint16_t>(i % 128));
    m.setPointField_uint8(i, "color_r", static_cast<uint8_t>(i % 251));
    m.setPointField_uint32(i, "id", static_cast<uint32_t>(i * 7919));
  }
  return m;
}

template <typename T>
void expectFieldEqual(const std::vector<T>& a, const std::vector<T>& b)
{
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); i++)
  {
    if constexpr (std::is_floating_point_v<T>)
    {
      if (std::isnan(a[i]))
      {
        EXPECT_TRUE(std::isnan(b[i]));
        continue;
      }
    }
    ASSERT_EQ(a[i], b[i]) << "i=" << i;
  }
}

template <typename T, typename A>
std::vector<T> asVector(const std::vector<T, A>* v)
{
  return v ? std::vector<T>(v->begin(), v->end()) : std::vector<T>();
}

void expectGenericFieldsEqual(const CGenericPointsMap& a, const CGenericPointsMap& b)
{
  for (const char* f : {"intensity", "t"})
  {
    expectFieldEqual(
        asVector(a.getPointsBufferRef_float_field(f)),
        asVector(b.getPointsBufferRef_float_field(f)));
  }
  expectFieldEqual(
      asVector(a.getPointsBufferRef_double_field("gps_time")),
      asVector(b.getPointsBufferRef_double_field("gps_time")));
  expectFieldEqual(
      asVector(a.getPointsBufferRef_uint16_field("ring")),
      asVector(b.getPointsBufferRef_uint16_field("ring")));
  expectFieldEqual(
      asVector(a.getPointsBufferRef_uint8_field("color_r")),
      asVector(b.getPointsBufferRef_uint8_field("color_r")));
  expectFieldEqual(
      asVector(a.getPointsBufferRef_uint32_field("id")),
      asVector(b.getPointsBufferRef_uint32_field("id")));
}

// Float rounding of the quantized values, on top of half the resolution:
constexpr float kFloatTolerance = 2e-5f;

}  // namespace

TEST(CPointsMap_cloudini, RawRoundTripIsExact)
{
  CPointsMap::TSerializationOptions o;
  o.use_cloudini = false;
  const ScopedOptions scoped(o);

  for (size_t n : {0UL, 5UL, 1000UL})
  {
    CSimplePointsMap m;
    fillRandomXYZ(m, n);
    CSimplePointsMap m2;
    deserialize(serialize(m), m2);
    expectXYZNear(m, m2, 0);

    CGenericPointsMap g = makeGenericMap(n);
    CGenericPointsMap g2;
    deserialize(serialize(g), g2);
    expectXYZNear(g, g2, 0);
    expectGenericFieldsEqual(g, g2);
  }
}

TEST(CPointsMap_cloudini, SimplePointsMapRoundTrip)
{
  if (!CPointsMap::hasCloudini())
  {
    GTEST_SKIP() << "Built without Cloudini";
  }
  for (const float res : {1e-4f, 1e-3f})
  {
    CPointsMap::TSerializationOptions o;
    o.xyz_resolution = res;
    o.cloudini_min_points = 0;
    const ScopedOptions scoped(o);

    // Small clouds are coded in this thread, large ones in parallel:
    for (size_t n : {0UL, 1UL, 1000UL, 300'000UL})
    {
      CSimplePointsMap m;
      fillRandomXYZ(m, n);
      const auto data = serialize(m);
      if (n == 300'000UL)
      {
        EXPECT_LT(data.size(), n * 3 * sizeof(float));
      }
      CSimplePointsMap m2;
      deserialize(data, m2);
      expectXYZNear(m, m2, 0.5f * res + kFloatTolerance);
    }
  }
}

TEST(CPointsMap_cloudini, GenericPointsMapRoundTrip)
{
  if (!CPointsMap::hasCloudini())
  {
    GTEST_SKIP() << "Built without Cloudini";
  }
  CPointsMap::TSerializationOptions o;  // defaults: other fields exact
  o.cloudini_min_points = 0;
  const ScopedOptions scoped(o);

  for (size_t n : {0UL, 1000UL, 300'000UL})
  {
    CGenericPointsMap g = makeGenericMap(n);
    if (n > 10)
    {
      g.setPointField_float(3, "intensity", std::numeric_limits<float>::quiet_NaN());
      g.setPointField_float(4, "t", std::numeric_limits<float>::infinity());
    }
    CGenericPointsMap g2;
    deserialize(serialize(g), g2);
    expectXYZNear(g, g2, 0.5f * o.xyz_resolution + kFloatTolerance);
    expectGenericFieldsEqual(g, g2);
  }
}

TEST(CPointsMap_cloudini, FieldResolutions)
{
  if (!CPointsMap::hasCloudini())
  {
    GTEST_SKIP() << "Built without Cloudini";
  }
  CPointsMap::TSerializationOptions o;
  o.cloudini_min_points = 0;
  o.field_resolutions["intensity"] = 0.01f;
  // Infinite values cannot be quantized: this field is stored exactly instead.
  o.field_resolutions["t"] = 1e-3f;
  const ScopedOptions scoped(o);

  CGenericPointsMap g = makeGenericMap(5000);
  g.setPointField_float(4, "t", std::numeric_limits<float>::infinity());
  CGenericPointsMap g2;
  deserialize(serialize(g), g2);

  const auto& i1 = *g.getPointsBufferRef_float_field("intensity");
  const auto& i2 = *g2.getPointsBufferRef_float_field("intensity");
  ASSERT_EQ(i1.size(), i2.size());
  for (size_t i = 0; i < i1.size(); i++)
  {
    ASSERT_NEAR(i1[i], i2[i], 0.005f + 1e-6f);
  }
  expectFieldEqual(
      asVector(g.getPointsBufferRef_float_field("t")),
      asVector(g2.getPointsBufferRef_float_field("t")));
}

TEST(CPointsMap_cloudini, EncodingByte)
{
  CPointsMap::TSerializationOptions o;
  o.use_cloudini = false;
  const ScopedOptions scoped(o);

  CSimplePointsMap m;
  fillRandomXYZ(m, 1000);
  const auto v10 = serialize(m);

  // Turn the v10 stream into a v11 one with the given encoding byte, which goes
  // right after the version number:
  const std::string name = CSimplePointsMap::GetRuntimeClassIdStatic().className;
  const auto it = std::search(v10.begin(), v10.end(), name.begin(), name.end());
  ASSERT_TRUE(it != v10.end());
  const auto versionPos = static_cast<size_t>(it - v10.begin()) + name.size();
  ASSERT_EQ(v10.at(versionPos), 10);
  auto makeV11 = [&](uint8_t encoding)
  {
    auto data = v10;
    data[versionPos] = 11;
    data.insert(data.begin() + static_cast<std::ptrdiff_t>(versionPos + 1), encoding);
    return data;
  };

  CSimplePointsMap m2;
  deserialize(makeV11(0), m2);  // raw
  expectXYZNear(m, m2, 0);

  EXPECT_ANY_THROW(deserialize(makeV11(200), m2));

  if (!CPointsMap::hasCloudini())
  {
    EXPECT_ANY_THROW(deserialize(makeV11(1), m2));
  }
}

TEST(CPointsMap_cloudini, SmallCloudsStayRaw)
{
  CPointsMap::TSerializationOptions o;
  o.cloudini_min_points = 2000;
  const ScopedOptions scoped(o);

  CSimplePointsMap m;
  fillRandomXYZ(m, 1000);
  const auto withThreshold = serialize(m);

  CPointsMap::serializationOptions().use_cloudini = false;
  EXPECT_EQ(withThreshold, serialize(m));
}

// The serialization version, stored right after the class name:
template <class MAP>
int serializedVersion(size_t numPoints)
{
  MAP m;
  fillRandomXYZ(m, numPoints);
  const auto data = serialize(m);
  const std::string name = MAP::GetRuntimeClassIdStatic().className;
  const auto it = std::search(data.begin(), data.end(), name.begin(), name.end());
  if (it == data.end() || it + static_cast<std::ptrdiff_t>(name.size()) == data.end())
  {
    return -1;
  }
  return *(it + static_cast<std::ptrdiff_t>(name.size()));
}

TEST(CPointsMap_cloudini, RawKeepsLegacyVersion)
{
  CPointsMap::TSerializationOptions o;
  o.use_cloudini = false;
  const ScopedOptions scoped(o);

  // Older MRPT versions can read uncompressed maps:
  EXPECT_EQ(serializedVersion<CSimplePointsMap>(100), 10);
  EXPECT_EQ(serializedVersion<CGenericPointsMap>(100), 4);

  if (CPointsMap::hasCloudini())
  {
    CPointsMap::serializationOptions().use_cloudini = true;
    CPointsMap::serializationOptions().cloudini_min_points = 0;
    EXPECT_EQ(serializedVersion<CSimplePointsMap>(100), 11);
    EXPECT_EQ(serializedVersion<CGenericPointsMap>(100), 5);
  }
}
