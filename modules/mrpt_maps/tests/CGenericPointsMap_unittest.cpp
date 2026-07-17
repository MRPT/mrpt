/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

// Unit tests for CGenericPointsMap field-order stability,
// thread-safety of the field registration API, and basic operations.

#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/serialization/CArchive.h>
#include <unistd.h>

#include <atomic>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

using mrpt::maps::CGenericPointsMap;
using mrpt::maps::CPointsMap;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

/// Build a small map with known field values so tests can assert on content.
static CGenericPointsMap makeTestMap()
{
  CGenericPointsMap m;
  m.registerField_float("intensity");
  m.registerField_float("t");
  m.registerField_uint16("ring");
  m.registerField_uint8("color_r");
  m.registerField_uint32("rgba");
  m.registerField_double("gps_time");

  const size_t N = 5;
  m.resize(N);
  for (size_t i = 0; i < N; ++i)
  {
    m.setPointFast(i, float(i), float(i * 10), float(i * 100));
    m.setPointField_float(i, "intensity", float(i) + 0.5f);
    m.setPointField_float(i, "t", float(i) * 0.001f);
    m.setPointField_uint16(i, "ring", static_cast<uint16_t>(i + 1));
    m.setPointField_uint8(i, "color_r", static_cast<uint8_t>(i * 50));
    m.setPointField_uint32(i, "rgba", static_cast<uint32_t>(i) * 100000u + 1u);
    m.setPointField_double(i, "gps_time", 1000.0 + static_cast<double>(i));
  }
  return m;
}

// =========================================================================
//  Thread safety: field keys survive cross-thread transfer
// =========================================================================

TEST(CGenericPointsMap, FieldKeysValidAfterCreatingThreadExits)
{
  std::unique_ptr<CGenericPointsMap> mapPtr;

  std::thread builder(
      [&mapPtr]()
      {
        auto m = std::make_unique<CGenericPointsMap>();
        m->registerField_float("intensity");
        m->registerField_uint16("ring");
        m->registerField_uint8("label");
        m->registerField_double("gps_time");

        m->resize(3);
        for (size_t i = 0; i < 3; ++i)
        {
          m->setPointFast(i, float(i), 0, 0);
          m->setPointField_float(i, "intensity", float(i) + 0.1f);
          m->setPointField_uint16(i, "ring", uint16_t(i));
          m->setPointField_uint8(i, "label", uint8_t(i + 10));
          m->setPointField_double(i, "gps_time", 100.0 + static_cast<double>(i));
        }
        mapPtr = std::move(m);
      });
  builder.join();

  ASSERT_NE(mapPtr, nullptr);
  const auto& m = *mapPtr;

  EXPECT_TRUE(m.hasPointField("intensity"));
  EXPECT_TRUE(m.hasPointField("ring"));
  EXPECT_TRUE(m.hasPointField("label"));
  EXPECT_TRUE(m.hasPointField("gps_time"));

  EXPECT_FLOAT_EQ(m.getPointField_float(1, "intensity"), 1.1f);
  EXPECT_EQ(m.getPointField_uint16(2, "ring"), 2u);
  EXPECT_EQ(m.getPointField_uint8(0, "label"), 10u);
  EXPECT_DOUBLE_EQ(m.getPointField_double(1, "gps_time"), 101.0);

  const auto fnames = m.getPointFieldNames_float();
  EXPECT_FALSE(fnames.empty());
}

TEST(CGenericPointsMap, DeserializeOnDifferentThread)
{
  mrpt::io::CMemoryStream buf;
  {
    const auto src = makeTestMap();
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }

  std::shared_ptr<CGenericPointsMap> mapPtr;
  buf.Seek(0);
  std::thread loader(
      [&]()
      {
        auto ar = mrpt::serialization::archiveFrom(buf);
        mrpt::serialization::CSerializable::Ptr obj;
        ar >> obj;
        mapPtr = std::dynamic_pointer_cast<CGenericPointsMap>(obj);
      });
  loader.join();

  ASSERT_NE(mapPtr, nullptr);
  const auto& m = *mapPtr;

  EXPECT_TRUE(m.hasPointField("intensity"));
  EXPECT_TRUE(m.hasPointField("t"));
  EXPECT_TRUE(m.hasPointField("ring"));
  EXPECT_TRUE(m.hasPointField("color_r"));
  EXPECT_TRUE(m.hasPointField("rgba"));
  EXPECT_TRUE(m.hasPointField("gps_time"));
  EXPECT_FLOAT_EQ(m.getPointField_float(2, "intensity"), 2.5f);
  EXPECT_EQ(m.getPointField_uint32(3, "rgba"), 300001u);
}

TEST(CGenericPointsMap, CopyAssignAcrossThreads)
{
  const auto src = makeTestMap();
  std::unique_ptr<CGenericPointsMap> dstPtr;

  std::thread copier([&]() { dstPtr = std::make_unique<CGenericPointsMap>(src); });
  copier.join();

  ASSERT_NE(dstPtr, nullptr);
  EXPECT_TRUE(dstPtr->hasPointField("intensity"));
  EXPECT_FLOAT_EQ(dstPtr->getPointField_float(0, "intensity"), 0.5f);
  EXPECT_DOUBLE_EQ(dstPtr->getPointField_double(2, "gps_time"), 1002.0);
}

// =========================================================================
//  reserveField / resizeField on unregistered fields must throw
// =========================================================================

TEST(CGenericPointsMap, ReserveFieldUnregisteredFieldThrows)
{
  CGenericPointsMap m;
  m.registerField_float("intensity");
  m.resize(5);

  EXPECT_ANY_THROW(m.reserveField_float("bogus", 100));
  EXPECT_ANY_THROW(m.reserveField_double("bogus", 100));
  EXPECT_ANY_THROW(m.reserveField_uint16("bogus", 100));
  EXPECT_ANY_THROW(m.reserveField_uint8("bogus", 100));
}

TEST(CGenericPointsMap, ResizeFieldUnregisteredFieldThrows)
{
  CGenericPointsMap m;
  m.registerField_float("intensity");
  m.resize(5);

  EXPECT_ANY_THROW(m.resizeField_float("bogus", 100));
  EXPECT_ANY_THROW(m.resizeField_double("bogus", 100));
  EXPECT_ANY_THROW(m.resizeField_uint16("bogus", 100));
  EXPECT_ANY_THROW(m.resizeField_uint8("bogus", 100));
}

// =========================================================================
//  getPointAllFieldsFast / setPointAllFieldsFast round-trip
// =========================================================================

TEST(CGenericPointsMap, AllFieldsSelfRoundTrip)
{
  CGenericPointsMap m;
  for (int i = 0; i < 20; ++i) m.registerField_float("field_" + std::to_string(i));

  m.resize(1);
  for (int i = 0; i < 20; ++i)
    m.setPointField_float(0, "field_" + std::to_string(i), float(i) + 0.25f);

  m.setPointFast(0, 1.0f, 2.0f, 3.0f);

  std::vector<float> blob;
  m.getPointAllFieldsFast(0, blob);

  // Wipe values.
  for (int i = 0; i < 20; ++i) m.setPointField_float(0, "field_" + std::to_string(i), 0.0f);

  // Restore from blob.
  m.setPointAllFieldsFast(0, blob);

  for (int i = 0; i < 20; ++i)
  {
    EXPECT_FLOAT_EQ(m.getPointField_float(0, "field_" + std::to_string(i)), float(i) + 0.25f)
        << "Mismatch at field_" << i;
  }
}

// =========================================================================
//  Copy-constructed map must preserve field order
// =========================================================================

TEST(CGenericPointsMap, CopyPreservesFieldOrder)
{
  const auto src = makeTestMap();
  std::vector<float> blobSrc;
  src.getPointAllFieldsFast(2, blobSrc);

  CGenericPointsMap dst(src);
  std::vector<float> blobDst;
  dst.getPointAllFieldsFast(2, blobDst);

  ASSERT_EQ(blobSrc.size(), blobDst.size());
  for (size_t i = 0; i < blobSrc.size(); ++i)
  {
    EXPECT_FLOAT_EQ(blobSrc[i], blobDst[i]) << "Mismatch at blob index " << i;
  }
}

// =========================================================================
//  Serialize -> deserialize must preserve field order
// =========================================================================

TEST(CGenericPointsMap, SerializeDeserializePreservesFieldOrder)
{
  const auto src = makeTestMap();
  std::vector<float> blobSrc;
  src.getPointAllFieldsFast(3, blobSrc);

  mrpt::io::CMemoryStream buf;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    ar << src;
  }
  buf.Seek(0);

  CGenericPointsMap dst;
  {
    auto ar = mrpt::serialization::archiveFrom(buf);
    mrpt::serialization::CSerializable::Ptr obj;
    ar >> obj;
    ASSERT_NE(obj, nullptr);
    dst = *dynamic_cast<CGenericPointsMap*>(obj.get());
  }

  std::vector<float> blobDst;
  dst.getPointAllFieldsFast(3, blobDst);

  ASSERT_EQ(blobSrc.size(), blobDst.size());
  for (size_t i = 0; i < blobSrc.size(); ++i)
  {
    EXPECT_FLOAT_EQ(blobSrc[i], blobDst[i]) << "Mismatch at blob index " << i;
  }
}

// =========================================================================
//  General regression: field access basics
// =========================================================================

TEST(CGenericPointsMap, RegisterAndAccessAllTypes)
{
  CGenericPointsMap m;
  m.registerField_float("f1");
  m.registerField_double("d1");
  m.registerField_uint16("u16");
  m.registerField_uint8("u8");
  m.registerField_uint32("u32");

  m.resize(2);
  m.setPointFast(0, 1.0f, 2.0f, 3.0f);
  m.setPointField_float(0, "f1", 1.5f);
  m.setPointField_double(0, "d1", 2.5);
  m.setPointField_uint16(0, "u16", 300);
  m.setPointField_uint8(0, "u8", 42);
  m.setPointField_uint32(0, "u32", 4000000000u);

  EXPECT_FLOAT_EQ(m.getPointField_float(0, "f1"), 1.5f);
  EXPECT_DOUBLE_EQ(m.getPointField_double(0, "d1"), 2.5);
  EXPECT_EQ(m.getPointField_uint16(0, "u16"), 300);
  EXPECT_EQ(m.getPointField_uint8(0, "u8"), 42);
  EXPECT_EQ(m.getPointField_uint32(0, "u32"), 4000000000u);

  // Field-name enumeration includes the uint32 channel:
  const auto u32names = m.getPointFieldNames_uint32();
  EXPECT_EQ(u32names.size(), 1u);
  EXPECT_EQ(u32names.at(0), "u32");

  // reserve/resize on a registered uint32 field does not throw:
  EXPECT_NO_THROW(m.reserveField_uint32("u32", 100));
  EXPECT_NO_THROW(m.resizeField_uint32("u32", 4));
  EXPECT_ANY_THROW(m.reserveField_uint32("bogus", 100));
  EXPECT_ANY_THROW(m.resizeField_uint32("bogus", 100));

  // unregister removes the uint32 field:
  EXPECT_TRUE(m.unregisterField("u32"));
  EXPECT_FALSE(m.hasPointField("u32"));
}

TEST(CGenericPointsMap, UnregisterFieldRemovesData)
{
  CGenericPointsMap m;
  m.registerField_float("toRemove");
  m.registerField_float("toKeep");
  m.resize(3);
  m.setPointField_float(0, "toRemove", 99.0f);
  m.setPointField_float(0, "toKeep", 42.0f);

  EXPECT_TRUE(m.unregisterField("toRemove"));
  EXPECT_FALSE(m.hasPointField("toRemove"));
  EXPECT_TRUE(m.hasPointField("toKeep"));

  // Re-register must work and start fresh.
  m.registerField_float("toRemove");
  EXPECT_FLOAT_EQ(m.getPointField_float(0, "toRemove"), 0.0f);
}

TEST(CGenericPointsMap, ClearAndReuse)
{
  CGenericPointsMap m;
  m.registerField_float("f1");
  m.resize(5);
  m.clear();

  EXPECT_FALSE(m.hasPointField("f1"));
  EXPECT_EQ(m.size(), 0u);

  m.registerField_float("f2");
  m.resize(2);
  m.setPointField_float(0, "f2", 7.0f);
  EXPECT_FLOAT_EQ(m.getPointField_float(0, "f2"), 7.0f);
}

// =========================================================================
//  Stress: many threads create and destroy maps concurrently
// =========================================================================

TEST(CGenericPointsMap, ConcurrentCreateDestroy)
{
  constexpr int kThreads = 8;
  constexpr int kIterations = 200;
  std::atomic<int> failures{0};

  auto worker = [&](int id)
  {
    for (int iter = 0; iter < kIterations; ++iter)
    {
      try
      {
        CGenericPointsMap m;
        const std::string fname = "field_" + std::to_string(id) + "_" + std::to_string(iter);
        m.registerField_float(fname);
        m.registerField_uint8("u8_" + std::to_string(id));
        m.resize(10);
        for (size_t i = 0; i < 10; ++i)
        {
          m.setPointFast(i, float(i), 0, 0);
          m.setPointField_float(i, fname, float(i) + 0.1f);
          m.setPointField_uint8(i, "u8_" + std::to_string(id), uint8_t(i));
        }

        for (size_t i = 0; i < 10; ++i)
        {
          const float v = m.getPointField_float(i, fname);
          if (std::abs(v - (float(i) + 0.1f)) > 1e-5f) ++failures;
        }
      }
      catch (...)
      {
        ++failures;
      }
    }
  };

  std::vector<std::thread> threads;
  threads.reserve(kThreads);
  for (int i = 0; i < kThreads; ++i) threads.emplace_back(worker, i);
  for (auto& t : threads) t.join();

  EXPECT_EQ(failures.load(), 0);
}

// =========================================================================
//  Produce on N threads, consume on main
// =========================================================================

TEST(CGenericPointsMap, ProduceOnThreadsConsumeOnMain)
{
  constexpr int kThreads = 4;
  std::vector<std::unique_ptr<CGenericPointsMap>> maps(kThreads);

  std::vector<std::thread> producers;
  producers.reserve(kThreads);
  for (int id = 0; id < kThreads; ++id)
  {
    producers.emplace_back(
        [&maps, id]()
        {
          auto m = std::make_unique<CGenericPointsMap>();
          const std::string fname = "val_" + std::to_string(id);
          m->registerField_float(fname);
          m->resize(5);
          for (size_t i = 0; i < 5; ++i)
          {
            m->setPointFast(i, float(i), 0, 0);
            m->setPointField_float(i, fname, float(id * 100 + i));
          }
          maps[id] = std::move(m);
        });
  }
  for (auto& t : producers) t.join();

  for (int id = 0; id < kThreads; ++id)
  {
    ASSERT_NE(maps[id], nullptr) << "Thread " << id << " didn't produce a map";
    const auto& m = *maps[id];
    const std::string fname = "val_" + std::to_string(id);
    EXPECT_TRUE(m.hasPointField(fname)) << "Field missing for thread " << id;
    for (size_t i = 0; i < 5; ++i)
    {
      EXPECT_FLOAT_EQ(m.getPointField_float(i, fname), float(id * 100 + i))
          << "Value mismatch: thread=" << id << " point=" << i;
    }
  }
}

// =========================================================================
//  Edge case: field names that are substrings of each other
// =========================================================================

TEST(CGenericPointsMap, SubstringFieldNames)
{
  CGenericPointsMap m;
  m.registerField_float("ring");
  m.registerField_float("ring_id");
  m.registerField_float("r");

  m.resize(1);
  m.setPointField_float(0, "ring", 1.0f);
  m.setPointField_float(0, "ring_id", 2.0f);
  m.setPointField_float(0, "r", 3.0f);

  EXPECT_FLOAT_EQ(m.getPointField_float(0, "ring"), 1.0f);
  EXPECT_FLOAT_EQ(m.getPointField_float(0, "ring_id"), 2.0f);
  EXPECT_FLOAT_EQ(m.getPointField_float(0, "r"), 3.0f);
}

// =========================================================================
//  Field registration corner cases
// =========================================================================

TEST(CGenericPointsMap, RegisterDuplicateFieldThrows)
{
  CGenericPointsMap m;
  ASSERT_TRUE(m.registerField_float("dup"));
  EXPECT_THROW(m.registerField_float("dup"), std::exception);

  ASSERT_TRUE(m.registerField_uint32("dup2"));
  EXPECT_THROW(m.registerField_uint32("dup2"), std::exception);
}

TEST(CGenericPointsMap, UnregisterNonexistentFieldReturnsFalse)
{
  CGenericPointsMap m;
  EXPECT_FALSE(m.unregisterField("does_not_exist"));

  m.registerField_float("f1");
  EXPECT_TRUE(m.unregisterField("f1"));
  // Removed already: a second attempt must fail:
  EXPECT_FALSE(m.unregisterField("f1"));
}

// =========================================================================
//  insertPointField_*() padding of out-of-sync field vectors
// =========================================================================

TEST(CGenericPointsMap, InsertPointFieldPadsMissingFloatValues)
{
  CGenericPointsMap m;
  m.registerField_float("f1");

  // Insert two XYZ points without touching "f1" at all, so its vector
  // lags behind m_x/m_y/m_z:
  m.insertPointFast(0, 0, 0);
  m.insertPointFast(1, 0, 0);

  // This call must zero-pad the missing first entry before appending:
  m.insertPointField_float("f1", 5.0f);

  EXPECT_FLOAT_EQ(m.getPointField_float(0, "f1"), 0.0f);
  EXPECT_FLOAT_EQ(m.getPointField_float(1, "f1"), 5.0f);
}

TEST(CGenericPointsMap, InsertPointFieldPadsMissingDoubleValues)
{
  CGenericPointsMap m;
  m.registerField_double("d1");

  m.insertPointFast(0, 0, 0);
  m.insertPointFast(1, 0, 0);
  m.insertPointFast(2, 0, 0);

  m.insertPointField_double("d1", 9.0);

  EXPECT_DOUBLE_EQ(m.getPointField_double(0, "d1"), 0.0);
  EXPECT_DOUBLE_EQ(m.getPointField_double(1, "d1"), 0.0);
  EXPECT_DOUBLE_EQ(m.getPointField_double(2, "d1"), 9.0);
}

// =========================================================================
//  getPointsBufferRef_*_field() const and non-const fallback chains
// =========================================================================

TEST(CGenericPointsMap, GetPointsBufferRefAllTypesConstAndNonConst)
{
  CGenericPointsMap m;
  m.registerField_float("f1");
  m.registerField_double("d1");
  m.registerField_uint16("u16");
  m.registerField_uint8("u8");
  m.registerField_uint32("u32");
  m.resize(3);
  m.setPointField_float(0, "f1", 1.5f);
  m.setPointField_double(0, "d1", 2.5);

  const CGenericPointsMap& cref = m;

  // Base XYZ field, resolved through CPointsMap::getPointsBufferRef_float_field:
  const auto* xBuf = cref.getPointsBufferRef_float_field("x");
  ASSERT_TRUE(xBuf != nullptr);
  EXPECT_EQ(xBuf->size(), 3u);

  const auto* fBuf = cref.getPointsBufferRef_float_field("f1");
  ASSERT_TRUE(fBuf != nullptr);
  EXPECT_FLOAT_EQ(fBuf->at(0), 1.5f);
  EXPECT_TRUE(cref.getPointsBufferRef_float_field("missing") == nullptr);

  const auto* dBuf = cref.getPointsBufferRef_double_field("d1");
  ASSERT_TRUE(dBuf != nullptr);
  EXPECT_DOUBLE_EQ(dBuf->at(0), 2.5);
  EXPECT_TRUE(cref.getPointsBufferRef_double_field("missing") == nullptr);

  EXPECT_TRUE(cref.getPointsBufferRef_uint16_field("u16") != nullptr);
  EXPECT_TRUE(cref.getPointsBufferRef_uint16_field("missing") == nullptr);
  EXPECT_TRUE(cref.getPointsBufferRef_uint8_field("u8") != nullptr);
  EXPECT_TRUE(cref.getPointsBufferRef_uint8_field("missing") == nullptr);
  EXPECT_TRUE(cref.getPointsBufferRef_uint32_field("u32") != nullptr);
  EXPECT_TRUE(cref.getPointsBufferRef_uint32_field("missing") == nullptr);

  // Non-const overloads:
  EXPECT_TRUE(m.getPointsBufferRef_double_field("d1") != nullptr);
  EXPECT_TRUE(m.getPointsBufferRef_uint16_field("u16") != nullptr);
  EXPECT_TRUE(m.getPointsBufferRef_uint8_field("u8") != nullptr);
  EXPECT_TRUE(m.getPointsBufferRef_uint32_field("u32") != nullptr);
  EXPECT_TRUE(m.getPointsBufferRef_double_field("missing") == nullptr);
}

// =========================================================================
//  PLY import/export round trip, exercising the color-field registration
//  performed by CGenericPointsMap's overrides of the PLY virtual methods.
// =========================================================================

TEST(CGenericPointsMap, PLYRoundTripPreservesIntensity)
{
  // Note: MRPT's PLY writer (mrpt::viz::PLY_import_export) only stores a
  // single "intensity" property per vertex, computed as the average of R/G/B
  // on export, and broadcasts it back equally to R/G/B on import. It does
  // NOT round-trip full per-channel RGB color.
  CGenericPointsMap src;
  src.registerField_float(CPointsMap::POINT_FIELD_COLOR_Rf);
  src.registerField_float(CPointsMap::POINT_FIELD_COLOR_Gf);
  src.registerField_float(CPointsMap::POINT_FIELD_COLOR_Bf);

  src.insertPointFast(1.0f, 2.0f, 3.0f);
  src.insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Rf, 0.1f);
  src.insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Gf, 0.2f);
  src.insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Bf, 0.3f);

  src.insertPointFast(4.0f, 5.0f, 6.0f);
  src.insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Rf, 0.4f);
  src.insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Gf, 0.5f);
  src.insertPointField_float(CPointsMap::POINT_FIELD_COLOR_Bf, 0.6f);

  static std::atomic<int> counter{0};
  const auto dir = std::filesystem::temp_directory_path();
  const std::string file =
      (dir / ("mrpt_CGenericPointsMap_unittest_" + std::to_string(static_cast<long>(getpid())) +
              "_" + std::to_string(counter++) + ".ply"))
          .string();

  ASSERT_TRUE(src.saveToPlyFile(file));

  CGenericPointsMap dst;
  ASSERT_TRUE(dst.loadFromPlyFile(file));

  ASSERT_EQ(dst.size(), 2u);
  EXPECT_TRUE(dst.hasColor_f());
  const float intensity0 = (0.1f + 0.2f + 0.3f) / 3.0f;
  const float intensity1 = (0.4f + 0.5f + 0.6f) / 3.0f;
  EXPECT_NEAR(dst.getPointField_float(0, CPointsMap::POINT_FIELD_COLOR_Rf), intensity0, 1e-3f);
  EXPECT_NEAR(dst.getPointField_float(0, CPointsMap::POINT_FIELD_COLOR_Gf), intensity0, 1e-3f);
  EXPECT_NEAR(dst.getPointField_float(0, CPointsMap::POINT_FIELD_COLOR_Bf), intensity0, 1e-3f);
  EXPECT_NEAR(dst.getPointField_float(1, CPointsMap::POINT_FIELD_COLOR_Rf), intensity1, 1e-3f);

  std::filesystem::remove(file);
}
