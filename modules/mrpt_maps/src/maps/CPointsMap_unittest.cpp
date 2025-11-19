/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CWeightedPointsMap.h>
#include <mrpt/poses/CPoint2D.h>

#include <array>
#include <sstream>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

constexpr size_t demo9_N = 9;
constexpr std::array<float, demo9_N> demo9_xs{0, 0, 0, 1, 1, 1, 2, 2, 2};
constexpr std::array<float, demo9_N> demo9_ys{0, 1, 2, 0, 1, 2, 0, 1, 2};
constexpr std::array<float, demo9_N> demo9_zs{0, 1, 2, 0, 1, 2, 0, 1, 2};

template <class MAP>
void load_demo_9pts_map(MAP& pts)
{
  pts.clear();
  for (size_t i = 0; i < demo9_N; i++) pts.insertPoint(demo9_xs[i], demo9_ys[i], demo9_zs[i]);
}

template <class MAP>
void do_test_insertPoints()
{
  // test 1: Insert and check expected values:
  {
    MAP pts;
    load_demo_9pts_map(pts);

    EXPECT_EQ(pts.size(), demo9_N);

    for (size_t i = 0; i < demo9_N; i++)
    {
      auto [x, y, z] = [&]()
      {
        float xi, yi, zi;
        pts.getPoint(i, xi, yi, zi);
        return std::tuple{xi, yi, zi};
      }();
      EXPECT_EQ(x, demo9_xs[i]);
      EXPECT_EQ(y, demo9_ys[i]);
      EXPECT_EQ(z, demo9_zs[i]);
    }
  }

  // test 2: Copy between maps
  {
    MAP pts1;
    load_demo_9pts_map(pts1);

    MAP pts2 = pts1;
    MAP pts3 = pts1;

    EXPECT_EQ(pts1.size(), pts2.size());
    EXPECT_EQ(pts2.size(), pts3.size());
    for (size_t i = 0; i < demo9_N; i++)
    {
      float x2, y2, z2;
      float x3, y3, z3;
      pts2.getPoint(i, x2, y2, z2);
      pts3.getPoint(i, x3, y3, z3);
      EXPECT_EQ(x2, x3);
      EXPECT_EQ(y2, y3);
      EXPECT_EQ(z2, z3);
    }
  }

  // test 3: Insert a map into another
  {
    MAP pts1;
    load_demo_9pts_map(pts1);

    EXPECT_EQ(pts1.size(), demo9_N);

    MAP pts;

    // Insert with += syntax;
    pts += pts1;

    // Insert via method:
    pts.insertAnotherMap(&pts1, {}, false /* filter out */);

    for (size_t i = 0; i < 2 * demo9_N; i++)
    {
      float x, y, z;
      pts.getPoint(i, x, y, z);
      EXPECT_EQ(x, demo9_xs[i % demo9_N]);
      EXPECT_EQ(y, demo9_ys[i % demo9_N]);
      EXPECT_EQ(z, demo9_zs[i % demo9_N]);
    }
    EXPECT_EQ(pts.size(), 2 * demo9_N);
  }

  // test 4: Insert a map into another with (0,0,0) filter:
  {
    MAP pts1;
    load_demo_9pts_map(pts1);

    EXPECT_EQ(pts1.size(), demo9_N);

    MAP pts;
    pts.insertAnotherMap(&pts1, {}, true /* filter out */);

    EXPECT_EQ(pts.size(), demo9_N - 1);
  }
}

template <class MAP>
void do_test_clipOutOfRangeInZ()
{
  MAP pts0;
  load_demo_9pts_map(pts0);

  // Clip: z=[-10,-1] -> 0 pts
  {
    MAP pts = pts0;
    pts.clipOutOfRangeInZ(-10, -1);
    EXPECT_EQ(pts.size(), 0u);
  }

  // Clip: z=[-10,10] -> 9 pts
  {
    MAP pts = pts0;
    pts.clipOutOfRangeInZ(-10, 10);
    EXPECT_EQ(pts.size(), 9u);
  }

  // Clip: z=[0.5,1.5] -> 3 pts
  {
    MAP pts = pts0;
    pts.clipOutOfRangeInZ(0.5, 1.5);
    EXPECT_EQ(pts.size(), 3u);
  }
}

template <class MAP>
void do_test_clipOutOfRange()
{
  MAP pts0;
  load_demo_9pts_map(pts0);

  // Clip:
  {
    TPoint2D pivot(0, 0);
    const float radius = 0.5;

    MAP pts = pts0;
    pts.clipOutOfRange(pivot, radius);
    EXPECT_EQ(pts.size(), 1u);
  }

  // Clip:
  {
    TPoint2D pivot(-10, -10);
    const float radius = 1;

    MAP pts = pts0;
    pts.clipOutOfRange(pivot, radius);
    EXPECT_EQ(pts.size(), 0u);
  }

  // Clip:
  {
    TPoint2D pivot(0, 0);
    const float radius = 1.1f;

    MAP pts = pts0;
    pts.clipOutOfRange(pivot, radius);
    EXPECT_EQ(pts.size(), 3u);
  }
}

template <class MAP>
void do_tests_loadSaveStreams()
{
  MAP pts0;
  load_demo_9pts_map(pts0);

  EXPECT_EQ(pts0.size(), 9u);

  auto lmb1 = [&]() -> std::string
  {
    std::stringstream ss;
    bool ret = pts0.save3D_to_text_stream(ss);
    EXPECT_TRUE(ret);
    return ss.str();
  };

  // Correct format:
  {
    MAP pts1;
    std::stringstream ss;
    ss.str(lmb1());
    bool ret = pts1.load3D_from_text_stream(ss);
    EXPECT_TRUE(ret);
    EXPECT_EQ(pts0.size(), pts1.size());
  }
  {
    MAP pts1;
    std::stringstream ss;
    ss.str("0 1\n1 2\n 3 4\n");
    bool ret = pts1.load2D_from_text_stream(ss);
    EXPECT_TRUE(ret);
    EXPECT_EQ(pts1.size(), 3u);
  }
  // Incorrect format:
  {
    MAP pts1;
    std::stringstream ss;
    ss.str("0 1\n1 2\n 3 4\n");
    std::string errMsg;
    bool ret = pts1.load3D_from_text_stream(ss, errMsg);
    EXPECT_FALSE(ret);
    EXPECT_EQ(pts1.size(), 0u);
  }
  {
    MAP pts1;
    std::stringstream ss;
    ss.str("0 1 3\n1 2 3 4\n3 4\n");
    std::string errMsg;
    bool ret = pts1.load3D_from_text_stream(ss, errMsg);
    EXPECT_FALSE(ret);
  }
  {
    MAP pts1;
    std::stringstream ss;
    ss.str("0 1\n1 2 3 4\n3 4\n");
    std::string errMsg;
    bool ret = pts1.load3D_from_text_stream(ss, errMsg);
    EXPECT_FALSE(ret);
  }
}

TEST(CSimplePointsMapTests, insertPoints) { do_test_insertPoints<CSimplePointsMap>(); }

TEST(CWeightedPointsMapTests, insertPoints) { do_test_insertPoints<CWeightedPointsMap>(); }

TEST(CColouredPointsMapTests, insertPoints) { do_test_insertPoints<CColouredPointsMap>(); }

TEST(CPointsMapXYZI, insertPoints) { do_test_insertPoints<CPointsMapXYZI>(); }

TEST(CSimplePointsMapTests, clipOutOfRangeInZ) { do_test_clipOutOfRangeInZ<CSimplePointsMap>(); }

TEST(CWeightedPointsMapTests, clipOutOfRangeInZ)
{
  do_test_clipOutOfRangeInZ<CWeightedPointsMap>();
}

TEST(CColouredPointsMapTests, clipOutOfRangeInZ)
{
  do_test_clipOutOfRangeInZ<CColouredPointsMap>();
}

TEST(CSimplePointsMapTests, clipOutOfRange) { do_test_clipOutOfRange<CSimplePointsMap>(); }

TEST(CWeightedPointsMapTests, clipOutOfRange) { do_test_clipOutOfRange<CWeightedPointsMap>(); }

TEST(CColouredPointsMapTests, clipOutOfRange) { do_test_clipOutOfRange<CColouredPointsMap>(); }

TEST(CSimplePointsMapTests, loadSaveStreams) { do_tests_loadSaveStreams<CSimplePointsMap>(); }

TEST(CWeightedPointsMapTests, loadSaveStreams) { do_tests_loadSaveStreams<CWeightedPointsMap>(); }

TEST(CColouredPointsMapTests, loadSaveStreams) { do_tests_loadSaveStreams<CColouredPointsMap>(); }

// ----------------------------------------------------------------------
// Tests for CGenericPointsMap custom field capabilities
// ----------------------------------------------------------------------

TEST(CGenericPointsMap, CustomFieldsLifecycle)
{
  CGenericPointsMap pts;

  // 1. Registration
  EXPECT_TRUE(pts.registerField_float("pressure"));
  EXPECT_TRUE(pts.registerField_double("timestamp"));
  EXPECT_TRUE(pts.registerField_uint16("class_id"));

  // Ensure duplicate registration returns true (idempotent) or handles gracefully,
  // but strict check: hasPointField should be true.
  EXPECT_TRUE(pts.hasPointField("pressure"));
  EXPECT_TRUE(pts.hasPointField("timestamp"));
  EXPECT_TRUE(pts.hasPointField("class_id"));
  EXPECT_FALSE(pts.hasPointField("non_existent"));

  // Check field name listings
  {
    const auto f_names = pts.getPointFieldNames_float();
    const auto d_names = pts.getPointFieldNames_double();
    const auto u_names = pts.getPointFieldNames_uint16();

    // Note: "x", "y", "z" are always in float names
    EXPECT_NE(std::find(f_names.begin(), f_names.end(), "pressure"), f_names.end());
    EXPECT_NE(std::find(d_names.begin(), d_names.end(), "timestamp"), d_names.end());
    EXPECT_NE(std::find(u_names.begin(), u_names.end(), "class_id"), u_names.end());
  }

  // 2. Insertion (Synchronized)
  const size_t N = 5;
  for (size_t i = 0; i < N; i++)
  {
    // Standard XYZ insert
    pts.insertPointFast(static_cast<float>(i), 0.0f, 0.0f);

    // Custom fields insert (Must match size of XYZ)
    pts.insertPointField_float("pressure", static_cast<float>(i) * 10);
    pts.insertPointField_double("timestamp", static_cast<double>(i) * 1000.0);
    pts.insertPointField_uint16("class_id", static_cast<uint16_t>(i + 1));
  }

  EXPECT_EQ(pts.size(), N);

  // 3. Random Access (Getters)
  for (size_t i = 0; i < N; i++)
  {
    EXPECT_FLOAT_EQ(pts.getPointField_float(i, "pressure"), static_cast<float>(i * 10));
    EXPECT_DOUBLE_EQ(pts.getPointField_double(i, "timestamp"), static_cast<double>(i * 1000.0));
    EXPECT_EQ(pts.getPointField_uint16(i, "class_id"), static_cast<uint16_t>(i + 1));
  }

  // 4. Random Access (Setters)
  pts.setPointField_float(0, "pressure", 99.9f);
  pts.setPointField_double(0, "timestamp", 888.88);
  pts.setPointField_uint16(0, "class_id", 55);

  EXPECT_FLOAT_EQ(pts.getPointField_float(0, "pressure"), 99.9f);
  EXPECT_DOUBLE_EQ(pts.getPointField_double(0, "timestamp"), 888.88);
  EXPECT_EQ(pts.getPointField_uint16(0, "class_id"), 55);

  // 5. Resize behavior
  // Resizing should resize auxiliary fields and initialize new elements to 0
  pts.resize(N + 2);
  EXPECT_EQ(pts.size(), N + 2);
  // Check old value preserved
  EXPECT_EQ(pts.getPointField_uint16(0, "class_id"), 55);
  // Check new value is zero-initialized
  EXPECT_EQ(pts.getPointField_float(N, "pressure"), 0.0f);
  EXPECT_EQ(pts.getPointField_uint16(N + 1, "class_id"), 0);

  // 6. Unregistration
  EXPECT_TRUE(pts.unregisterField("pressure"));
  EXPECT_FALSE(pts.hasPointField("pressure"));
  // Trying to access a removed field: Returns 0 if field does not exist
  EXPECT_EQ(pts.getPointField_float(0, "pressure"), 0.0f);
}

TEST(CGenericPointsMap, DeepCopy)
{
  CGenericPointsMap pts1;
  pts1.registerField_float("intensity");

  pts1.insertPointFast(1, 1, 1);
  pts1.insertPointField_float("intensity", 0.5f);

  pts1.insertPointFast(2, 2, 2);
  pts1.insertPointField_float("intensity", 1.0f);

  // Copy Constructor
  const CGenericPointsMap pts2 = pts1;

  EXPECT_EQ(pts1.size(), pts2.size());
  EXPECT_TRUE(pts2.hasPointField("intensity"));
  EXPECT_FLOAT_EQ(pts2.getPointField_float(1, "intensity"), 1.0f);

  // Modify original, ensure copy is independent
  pts1.setPointField_float(1, "intensity", 0.0f);
  EXPECT_FLOAT_EQ(pts1.getPointField_float(1, "intensity"), 0.0f);
  EXPECT_FLOAT_EQ(pts2.getPointField_float(1, "intensity"), 1.0f);
}

TEST(CGenericPointsMap, DirectVectorAccess)
{
  CGenericPointsMap pts;
  pts.registerField_float("temperature");

  pts.resize(10);

  // Access via reference
  auto* vec = pts.getPointsBufferRef_float_field("temperature");
  ASSERT_TRUE(vec != nullptr);
  ASSERT_EQ(vec->size(), 10u);

  // Modify vector directly
  (*vec)[5] = 37.5f;

  // Read back via API
  EXPECT_FLOAT_EQ(pts.getPointField_float(5, "temperature"), 37.5f);
}

// Verify that standard maps (CSimplePointsMap) behave correctly (reject custom fields)
TEST(CSimplePointsMapTests, RejectCustomFields)
{
  CSimplePointsMap pts;

  // Should return false as CSimplePointsMap doesn't support dynamic fields
  EXPECT_FALSE(pts.registerField_float("my_custom_float"));
  EXPECT_FALSE(pts.registerField_double("my_custom_double"));
  EXPECT_FALSE(pts.registerField_uint16("my_custom_uint"));

  // Should return false/empty
  EXPECT_FALSE(pts.hasPointField("my_custom_float"));

  // Standard fields should still work
  EXPECT_TRUE(pts.hasPointField("x"));
  EXPECT_TRUE(pts.hasPointField("y"));
  EXPECT_TRUE(pts.hasPointField("z"));
}
