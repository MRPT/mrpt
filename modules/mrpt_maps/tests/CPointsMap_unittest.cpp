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
#include <mrpt/maps/CGenericPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPoint2D.h>

#include <array>
#include <sstream>
#include <stdexcept>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

namespace
{
constexpr size_t demo9_N = 9;
constexpr std::array<float, demo9_N> demo9_xs{0, 0, 0, 1, 1, 1, 2, 2, 2};
constexpr std::array<float, demo9_N> demo9_ys{0, 1, 2, 0, 1, 2, 0, 1, 2};
constexpr std::array<float, demo9_N> demo9_zs{0, 1, 2, 0, 1, 2, 0, 1, 2};

template <class MAP>
[[nodiscard]] MAP load_demo_9pts_map()
{
  MAP pts;
  for (size_t i = 0; i < demo9_N; i++)
  {
    pts.insertPoint(demo9_xs[i], demo9_ys[i], demo9_zs[i]);
  }
  return pts;
}

template <class MAP>
void do_test_insertPoints()
{
  // test 1: Insert and check expected values:
  {
    const MAP pts = load_demo_9pts_map<MAP>();

    EXPECT_EQ(pts.size(), demo9_N);

    for (size_t i = 0; i < demo9_N; i++)
    {
      auto [x, y, z] = [&]()
      {
        mrpt::math::TPoint3Df pt;
        pts.getPoint(i, pt.x, pt.y, pt.z);
        return std::tuple{pt.x, pt.y, pt.z};
      }();
      EXPECT_EQ(x, demo9_xs[i]);
      EXPECT_EQ(y, demo9_ys[i]);
      EXPECT_EQ(z, demo9_zs[i]);
    }
  }

  // test 2: Copy between maps
  {
    const MAP pts1 = load_demo_9pts_map<MAP>();

    const MAP pts2 = pts1;  // NOLINT
    const MAP pts3 = pts1;  // NOLINT

    EXPECT_EQ(pts1.size(), pts2.size());
    EXPECT_EQ(pts2.size(), pts3.size());
    for (size_t i = 0; i < demo9_N; i++)
    {
      float x2, y2, z2;  // NOLINT
      float x3, y3, z3;  // NOLINT
      pts2.getPoint(i, x2, y2, z2);
      pts3.getPoint(i, x3, y3, z3);
      EXPECT_EQ(x2, x3);
      EXPECT_EQ(y2, y3);
      EXPECT_EQ(z2, z3);
    }
  }

  // test 3: Insert a map into another
  {
    const MAP pts1 = load_demo_9pts_map<MAP>();

    EXPECT_EQ(pts1.size(), demo9_N);

    MAP pts;

    // Insert with += syntax;
    pts += pts1;

    // Insert via method:
    pts.insertAnotherMap(&pts1, {}, false /* filter out */);

    for (size_t i = 0; i < 2 * demo9_N; i++)
    {
      float x, y, z;  // NOLINT
      pts.getPoint(i, x, y, z);
      EXPECT_EQ(x, demo9_xs[i % demo9_N]);
      EXPECT_EQ(y, demo9_ys[i % demo9_N]);
      EXPECT_EQ(z, demo9_zs[i % demo9_N]);
    }
    EXPECT_EQ(pts.size(), 2 * demo9_N);
  }

  // test 4: Insert a map into another with (0,0,0) filter:
  {
    const MAP pts1 = load_demo_9pts_map<MAP>();

    EXPECT_EQ(pts1.size(), demo9_N);

    MAP pts;
    pts.insertAnotherMap(&pts1, {}, true /* filter out */);

    EXPECT_EQ(pts.size(), demo9_N - 1);
  }
}

template <class MAP>
void do_test_clipOutOfRangeInZ()
{
  const MAP pts0 = load_demo_9pts_map<MAP>();

  // Clip: z=[-10,-1] -> 0 pts
  {
    MAP pts;
    pts0.clipOutOfRangeInZ(-10, -1, pts);
    EXPECT_EQ(pts.size(), 0u);
  }

  // Clip: z=[-10,10] -> 9 pts
  {
    MAP pts;
    pts0.clipOutOfRangeInZ(-10, 10, pts);
    EXPECT_EQ(pts.size(), 9u);
  }

  // Clip: z=[0.5,1.5] -> 3 pts
  {
    MAP pts;
    pts0.clipOutOfRangeInZ(0.5, 1.5, pts);
    EXPECT_EQ(pts.size(), 3u);
  }
}

template <class MAP>
void do_test_clipOutOfRange()
{
  const MAP pts0 = load_demo_9pts_map<MAP>();

  // Clip:
  {
    TPoint2D pivot(0, 0);
    const float radius = 0.5;

    MAP pts;
    pts0.clipOutOfRange(pivot, radius, pts);
    EXPECT_EQ(pts.size(), 1u);
  }

  // Clip:
  {
    TPoint2D pivot(-10, -10);
    const float radius = 1;

    MAP pts;
    pts0.clipOutOfRange(pivot, radius, pts);
    EXPECT_EQ(pts.size(), 0u);
  }

  // Clip:
  {
    TPoint2D pivot(0, 0);
    const float radius = 1.1f;

    MAP pts;
    pts0.clipOutOfRange(pivot, radius, pts);
    EXPECT_EQ(pts.size(), 3u);
  }
}

template <class MAP>
void do_tests_loadSaveStreams()
{
  const MAP pts0 = load_demo_9pts_map<MAP>();

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

}  // namespace

TEST(CSimplePointsMapTests, insertPoints) { do_test_insertPoints<CSimplePointsMap>(); }

TEST(CSimplePointsMapTests, clipOutOfRangeInZ) { do_test_clipOutOfRangeInZ<CSimplePointsMap>(); }

TEST(CSimplePointsMapTests, clipOutOfRange) { do_test_clipOutOfRange<CSimplePointsMap>(); }

TEST(CSimplePointsMapTests, loadSaveStreams) { do_tests_loadSaveStreams<CSimplePointsMap>(); }

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
    EXPECT_DOUBLE_EQ(pts.getPointField_double(i, "timestamp"), static_cast<double>(i) * 1000.0);
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

TEST(CSimplePointsMapTests, insert2DScan)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  // Insert the scan:
  mrpt::maps::CSimplePointsMap pnt;
  pnt.insertObservation(scan1);
  EXPECT_EQ(pnt.size(), 267UL);
}

TEST(CGenericPointsMapTests, insert2DScan)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  // Insert the scan:
  mrpt::maps::CGenericPointsMap pnt;
  pnt.registerField_float(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);

  pnt.insertObservation(scan1);
  ASSERT_EQUAL_(pnt.size(), 267UL);

  const auto* ts =
      pnt.getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);
  ASSERT_(ts != nullptr);

  EXPECT_EQ(ts->size(), pnt.size());
  EXPECT_NEAR((*ts).at(0), 0.0f, 1e-3f);
  EXPECT_NEAR(*(*ts).rbegin(), 0.025f, 1e-3f);
}

// ----------------------------------------------------------------------
// Tests for the KDTreeCapable / NearestNeighborsCapable interfaces, as
// implemented by CPointsMap over the 3x3x3 demo point cloud:
//   (0,0,0) (0,1,1) (0,2,2)
//   (1,0,0) (1,1,1) (1,2,2)
//   (2,0,0) (2,1,1) (2,2,2)
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, kdTreeClosestPoint2D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  float closestX, closestY, distSqr;
  const size_t idx = pts.kdTreeClosestPoint2D(0.1f, 0.1f, closestX, closestY, distSqr);

  // Closest point to (0.1,0.1) is (0,0,0), index 0:
  EXPECT_EQ(idx, 0u);
  EXPECT_FLOAT_EQ(closestX, 0.0f);
  EXPECT_FLOAT_EQ(closestY, 0.0f);
  EXPECT_NEAR(distSqr, 0.02f, 1e-5f);
}

TEST(CSimplePointsMapTests, kdTreeClosestPoint3D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  float closestX, closestY, closestZ, distSqr;
  const size_t idx =
      pts.kdTreeClosestPoint3D(2.1f, 2.1f, 2.1f, closestX, closestY, closestZ, distSqr);

  // Closest point to (2.1,2.1,2.1) is (2,2,2), the last point, index 8:
  EXPECT_EQ(idx, demo9_N - 1);
  EXPECT_FLOAT_EQ(closestX, 2.0f);
  EXPECT_FLOAT_EQ(closestY, 2.0f);
  EXPECT_FLOAT_EQ(closestZ, 2.0f);
  EXPECT_NEAR(distSqr, 0.03f, 1e-5f);
}

TEST(CSimplePointsMapTests, kdTreeNClosestPoint2D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  std::vector<float> xs, ys, dists;
  const auto idxs = pts.kdTreeNClosestPoint2D(0.0f, 0.0f, 3, xs, ys, dists);

  ASSERT_EQ(idxs.size(), 3u);
  ASSERT_EQ(xs.size(), 3u);
  ASSERT_EQ(ys.size(), 3u);
  ASSERT_EQ(dists.size(), 3u);

  // The query point (0,0) coincides with point index 0, so it must be the
  // first (closest) result, with zero distance:
  EXPECT_EQ(idxs[0], 0u);
  EXPECT_FLOAT_EQ(dists[0], 0.0f);

  // Distances must be sorted in increasing order:
  EXPECT_LE(dists[0], dists[1]);
  EXPECT_LE(dists[1], dists[2]);
}

TEST(CSimplePointsMapTests, kdTreeRadiusSearch2D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  std::vector<nanoflann::ResultItem<size_t, float>> results;
  // maxRadiusSqr=0.5: only the exact match (distSqr=0) is closer than the
  // next nearest points, which are all at distSqr=1:
  const size_t count = pts.kdTreeRadiusSearch2D(0.0f, 0.0f, 0.5f, results);

  EXPECT_EQ(count, 1u);
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].first, 0u);
}

TEST(CSimplePointsMapTests, nn_single_search_2D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  TPoint2Df result;
  float distSqr = 0;
  uint64_t resultIdx = 0;
  const bool found = pts.nn_single_search({0.1f, 0.1f}, result, distSqr, resultIdx);

  ASSERT_TRUE(found);
  EXPECT_EQ(resultIdx, 0u);
  EXPECT_FLOAT_EQ(result.x, 0.0f);
  EXPECT_FLOAT_EQ(result.y, 0.0f);
}

TEST(CSimplePointsMapTests, nn_single_search_3D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  TPoint3Df result;
  float distSqr = 0;
  uint64_t resultIdx = 0;
  const bool found = pts.nn_single_search({2.1f, 2.1f, 2.1f}, result, distSqr, resultIdx);

  ASSERT_TRUE(found);
  EXPECT_EQ(resultIdx, demo9_N - 1);
  EXPECT_FLOAT_EQ(result.x, 2.0f);
  EXPECT_FLOAT_EQ(result.y, 2.0f);
  EXPECT_FLOAT_EQ(result.z, 2.0f);
}

TEST(CSimplePointsMapTests, nn_multiple_search_2D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  std::vector<TPoint2Df> results;
  std::vector<float> dists;
  std::vector<uint64_t> idxs;
  pts.nn_multiple_search({0.0f, 0.0f}, 3, results, dists, idxs);

  ASSERT_EQ(results.size(), 3u);
  ASSERT_EQ(dists.size(), 3u);
  ASSERT_EQ(idxs.size(), 3u);

  // The query point coincides with point index 0:
  EXPECT_EQ(idxs[0], 0u);
  EXPECT_FLOAT_EQ(dists[0], 0.0f);
  EXPECT_LE(dists[0], dists[1]);
  EXPECT_LE(dists[1], dists[2]);
}

TEST(CSimplePointsMapTests, nn_radius_search_3D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  std::vector<TPoint3Df> results;
  std::vector<float> dists;
  std::vector<uint64_t> idxs;
  // search_radius_sqr=0.5: only the exact match (distSqr=0) is closer than
  // the next nearest points, which are all at distSqr=1:
  pts.nn_radius_search({0.0f, 0.0f, 0.0f}, 0.5f, results, dists, idxs, 0);

  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(idxs[0], 0u);
  ASSERT_EQ(idxs.size(), 1u);
  ASSERT_EQ(dists.size(), 1u);
  EXPECT_FLOAT_EQ(dists[0], 0.0f);
}

// ----------------------------------------------------------------------
// Tests for KDTreeCapable::kdtree_save_index_2D/3D() and
// kdtree_load_index_2D/3D(), over the same 3x3x3 demo point cloud.
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, kdtreeSaveLoadIndex3D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  std::stringstream ss;
  ASSERT_TRUE(pts.kdtree_save_index_3D(ss));

  // A fresh map with the identical points (same count) can load the index:
  auto pts2 = load_demo_9pts_map<CSimplePointsMap>();
  pts2.kdtree_load_index_3D(ss);

  float closestX, closestY, closestZ, distSqr;
  const size_t idx =
      pts2.kdTreeClosestPoint3D(2.1f, 2.1f, 2.1f, closestX, closestY, closestZ, distSqr);

  EXPECT_EQ(idx, demo9_N - 1);
  EXPECT_FLOAT_EQ(closestX, 2.0f);
  EXPECT_FLOAT_EQ(closestY, 2.0f);
  EXPECT_FLOAT_EQ(closestZ, 2.0f);
  EXPECT_NEAR(distSqr, 0.03f, 1e-5f);
}

TEST(CSimplePointsMapTests, kdtreeSaveLoadIndex2D)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  std::stringstream ss;
  ASSERT_TRUE(pts.kdtree_save_index_2D(ss));

  auto pts2 = load_demo_9pts_map<CSimplePointsMap>();
  pts2.kdtree_load_index_2D(ss);

  float closestX, closestY, distSqr;
  const size_t idx = pts2.kdTreeClosestPoint2D(0.1f, 0.1f, closestX, closestY, distSqr);

  EXPECT_EQ(idx, 0u);
  EXPECT_FLOAT_EQ(closestX, 0.0f);
  EXPECT_FLOAT_EQ(closestY, 0.0f);
  EXPECT_NEAR(distSqr, 0.02f, 1e-5f);
}

TEST(CSimplePointsMapTests, kdtreeSaveIndexAfterOtherDimensionQueried)
{
  // Regression test: querying one dimension first must not make the other
  // dimension's save_index() spuriously return false (shared uptodate flag).
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  float cx, cy, cz, distSqr;
  pts.kdTreeClosestPoint2D(0.1f, 0.1f, cx, cy, distSqr);

  std::stringstream ss;
  ASSERT_TRUE(pts.kdtree_save_index_3D(ss));

  auto pts2 = load_demo_9pts_map<CSimplePointsMap>();
  pts2.kdtree_load_index_3D(ss);
  const size_t idx = pts2.kdTreeClosestPoint3D(2.1f, 2.1f, 2.1f, cx, cy, cz, distSqr);
  EXPECT_EQ(idx, demo9_N - 1);
}

TEST(CSimplePointsMapTests, kdtreeSaveIndexEmptyMap)
{
  const CSimplePointsMap emptyPts;

  std::stringstream ss2D, ss3D;
  // No points: nothing is written, and the call returns false.
  EXPECT_FALSE(emptyPts.kdtree_save_index_2D(ss2D));
  EXPECT_FALSE(emptyPts.kdtree_save_index_3D(ss3D));
}

TEST(CSimplePointsMapTests, kdtreeLoadIndexPointCountMismatchThrows)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  std::stringstream ss;
  ASSERT_TRUE(pts.kdtree_save_index_3D(ss));

  // A map with a different point count must fail the point-count header
  // check performed by kdtree_load_index_3D().
  CSimplePointsMap mismatched;
  mismatched.insertPoint(0.0f, 0.0f, 0.0f);

  EXPECT_THROW(mismatched.kdtree_load_index_3D(ss), std::exception);
}
