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
#include <mrpt/math/KDTreeCapable.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationComment.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
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

namespace
{
/// Builds a unique path in the system temp directory for round-trip file tests.
std::string makeTempFilePath(const std::string& suffix)
{
  static std::atomic<int> counter{0};
  const auto dir = std::filesystem::temp_directory_path();
  const std::string fname = "mrpt_CPointsMap_unittest_" +
                            std::to_string(static_cast<long>(getpid())) + "_" +
                            std::to_string(counter++) + suffix;
  return (dir / fname).string();
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
#if !MRPT_NANOFLANN_HAS_KDTREE_SAVE_LOAD
  GTEST_SKIP() << "Requires nanoflann >= v1.5.0";
#endif
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
#if !MRPT_NANOFLANN_HAS_KDTREE_SAVE_LOAD
  GTEST_SKIP() << "Requires nanoflann >= v1.5.0";
#endif
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
#if !MRPT_NANOFLANN_HAS_KDTREE_SAVE_LOAD
  GTEST_SKIP() << "Requires nanoflann >= v1.5.0";
#endif
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
#if !MRPT_NANOFLANN_HAS_KDTREE_SAVE_LOAD
  GTEST_SKIP() << "Requires nanoflann >= v1.5.0";
#endif
  const CSimplePointsMap emptyPts;

  std::stringstream ss2D, ss3D;
  // No points: nothing is written, and the call returns false.
  EXPECT_FALSE(emptyPts.kdtree_save_index_2D(ss2D));
  EXPECT_FALSE(emptyPts.kdtree_save_index_3D(ss3D));
}

TEST(CSimplePointsMapTests, kdtreeLoadIndexPointCountMismatchThrows)
{
#if !MRPT_NANOFLANN_HAS_KDTREE_SAVE_LOAD
  GTEST_SKIP() << "Requires nanoflann >= v1.5.0";
#endif
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();

  std::stringstream ss;
  ASSERT_TRUE(pts.kdtree_save_index_3D(ss));

  // A map with a different point count must fail the point-count header
  // check performed by kdtree_load_index_3D().
  CSimplePointsMap mismatched;
  mismatched.insertPoint(0.0f, 0.0f, 0.0f);

  EXPECT_THROW(mismatched.kdtree_load_index_3D(ss), std::exception);
}

// ----------------------------------------------------------------------
// Tests for text file/stream save & load (2D and 3D formats).
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, save2DTextFileRoundTrip)
{
  const auto pts0 = load_demo_9pts_map<CSimplePointsMap>();
  const std::string file = makeTempFilePath("_2d.txt");

  ASSERT_TRUE(pts0.save2D_to_text_file(file));

  CSimplePointsMap pts1;
  ASSERT_TRUE(pts1.load2D_from_text_file(file));
  EXPECT_EQ(pts1.size(), pts0.size());

  for (size_t i = 0; i < pts1.size(); i++)
  {
    float x, y, z;
    pts1.getPoint(i, x, y, z);
    EXPECT_FLOAT_EQ(x, demo9_xs[i]);
    EXPECT_FLOAT_EQ(y, demo9_ys[i]);
    // z is not stored in the 2D format:
    EXPECT_FLOAT_EQ(z, 0.0f);
  }

  std::filesystem::remove(file);
}

TEST(CSimplePointsMapTests, save3DTextFileRoundTrip)
{
  const auto pts0 = load_demo_9pts_map<CSimplePointsMap>();
  const std::string file = makeTempFilePath("_3d.txt");

  ASSERT_TRUE(pts0.save3D_to_text_file(file));

  CSimplePointsMap pts1;
  ASSERT_TRUE(pts1.load3D_from_text_file(file));
  EXPECT_EQ(pts1.size(), pts0.size());

  for (size_t i = 0; i < pts1.size(); i++)
  {
    float x, y, z;
    pts1.getPoint(i, x, y, z);
    EXPECT_FLOAT_EQ(x, demo9_xs[i]);
    EXPECT_FLOAT_EQ(y, demo9_ys[i]);
    EXPECT_FLOAT_EQ(z, demo9_zs[i]);
  }

  std::filesystem::remove(file);
}

TEST(CSimplePointsMapTests, save2DToTextFileBadPathReturnsFalse)
{
  const auto pts0 = load_demo_9pts_map<CSimplePointsMap>();
  EXPECT_FALSE(pts0.save2D_to_text_file("/nonexistent_dir_xyz_abc/out.txt"));
}

TEST(CSimplePointsMapTests, save3DToTextFileBadPathReturnsFalse)
{
  const auto pts0 = load_demo_9pts_map<CSimplePointsMap>();
  EXPECT_FALSE(pts0.save3D_to_text_file("/nonexistent_dir_xyz_abc/out.txt"));
}

TEST(CSimplePointsMapTests, load2Dor3DFromTextFileNonexistentReturnsFalse)
{
  CSimplePointsMap pts;
  EXPECT_FALSE(pts.load2D_from_text_file("/nonexistent_dir_xyz_abc/in.txt"));
  EXPECT_FALSE(pts.load3D_from_text_file("/nonexistent_dir_xyz_abc/in.txt"));
}

TEST(CSimplePointsMapTests, load3DFromTextStreamErrorWithoutErrMsgGoesToStderr)
{
  // Same malformed content as the errMsg-based test, but calling the overload
  // without an output error string, to exercise the std::cerr fallback path.
  CSimplePointsMap pts;
  std::stringstream ss;
  ss.str("0 1\n1 2\n 3 4\n");
  const bool ret = pts.load3D_from_text_stream(ss);
  EXPECT_FALSE(ret);
  EXPECT_EQ(pts.size(), 0u);
}

// ----------------------------------------------------------------------
// Tests for changeCoordinatesReference() overloads.
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, changeCoordinatesReferencePose2D)
{
  CSimplePointsMap pts;
  pts.insertPoint(1.0f, 0.0f, 0.0f);

  pts.changeCoordinatesReference(CPose2D(2.0, 3.0, 0.0));

  float x, y, z;
  pts.getPoint(0, x, y, z);
  EXPECT_NEAR(x, 3.0f, 1e-5f);
  EXPECT_NEAR(y, 3.0f, 1e-5f);
  EXPECT_NEAR(z, 0.0f, 1e-5f);
}

TEST(CSimplePointsMapTests, changeCoordinatesReferencePose3D)
{
  CSimplePointsMap pts;
  pts.insertPoint(1.0f, 0.0f, 0.0f);

  pts.changeCoordinatesReference(CPose3D(2.0, 3.0, 4.0, 0.0, 0.0, 0.0));

  float x, y, z;
  pts.getPoint(0, x, y, z);
  EXPECT_NEAR(x, 3.0f, 1e-5f);
  EXPECT_NEAR(y, 3.0f, 1e-5f);
  EXPECT_NEAR(z, 4.0f, 1e-5f);
}

TEST(CSimplePointsMapTests, changeCoordinatesReferenceOtherMapAndBase)
{
  CSimplePointsMap other;
  other.insertPoint(1.0f, 0.0f, 0.0f);

  CSimplePointsMap pts;
  pts.insertPoint(-100.0f, -100.0f, -100.0f);  // Should be discarded by the copy.

  pts.changeCoordinatesReference(other, CPose3D(2.0, 3.0, 4.0, 0.0, 0.0, 0.0));

  ASSERT_EQ(pts.size(), 1u);
  float x, y, z;
  pts.getPoint(0, x, y, z);
  EXPECT_NEAR(x, 3.0f, 1e-5f);
  EXPECT_NEAR(y, 3.0f, 1e-5f);
  EXPECT_NEAR(z, 4.0f, 1e-5f);
}

// ----------------------------------------------------------------------
// Tests for determineMatching2D / determineMatching3D / compute3DMatchingRatio.
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, determineMatching2DFindsCorrespondences)
{
  CSimplePointsMap globalMap;
  globalMap.insertPoint(0.0f, 0.0f, 0.0f);
  globalMap.insertPoint(1.0f, 0.0f, 0.0f);
  globalMap.insertPoint(0.0f, 1.0f, 0.0f);

  CSimplePointsMap otherMap;
  otherMap.insertPoint(0.05f, 0.02f, 0.0f);
  otherMap.insertPoint(1.02f, -0.03f, 0.0f);

  mrpt::maps::TMatchingParams params;
  params.maxDistForCorrespondence = 0.5f;
  mrpt::maps::TMatchingExtraResults extraResults;
  mrpt::tfest::TMatchingPairList correspondences;

  globalMap.determineMatching2D(&otherMap, CPose2D(0, 0, 0), correspondences, params, extraResults);

  EXPECT_EQ(correspondences.size(), 2u);
  EXPECT_GT(extraResults.correspondencesRatio, 0.0f);
}

TEST(CSimplePointsMapTests, determineMatching2DAngularConstraintExtendsRange)
{
  // Note: determineMatching2D() has a coarse bounding-box overlap
  // pre-filter that is not expanded by maxDistForCorrespondence /
  // maxAngularDistForCorrespondence, so it can early-reject valid
  // correspondences when the two (possibly single-point) maps' bounding
  // boxes don't literally touch. A second global-map point is added here,
  // beyond the one actually expected to match, purely to make the two
  // maps' bounding boxes overlap so that pre-filter doesn't mask the
  // angular-distance behavior under test.
  CSimplePointsMap globalMap;
  globalMap.insertPoint(10.0f, 0.0f, 0.0f);
  globalMap.insertPoint(10.2f, 0.0f, 0.0f);

  CSimplePointsMap otherMap;
  otherMap.insertPoint(10.1f, 0.0f, 0.0f);  // 0.1 m away from the pivot-distant point.

  mrpt::maps::TMatchingParams paramsNoAngular;
  paramsNoAngular.maxDistForCorrespondence = 0.01f;
  paramsNoAngular.maxAngularDistForCorrespondence = 0.0f;
  mrpt::maps::TMatchingExtraResults extraNoAngular;
  mrpt::tfest::TMatchingPairList corrsNoAngular;
  globalMap.determineMatching2D(
      &otherMap, CPose2D(0, 0, 0), corrsNoAngular, paramsNoAngular, extraNoAngular);
  EXPECT_TRUE(corrsNoAngular.empty());

  mrpt::maps::TMatchingParams paramsAngular;
  paramsAngular.maxDistForCorrespondence = 0.01f;
  paramsAngular.maxAngularDistForCorrespondence = 0.02f;
  paramsAngular.angularDistPivotPoint = TPoint3D(0, 0, 0);
  mrpt::maps::TMatchingExtraResults extraAngular;
  mrpt::tfest::TMatchingPairList corrsAngular;
  globalMap.determineMatching2D(
      &otherMap, CPose2D(0, 0, 0), corrsAngular, paramsAngular, extraAngular);
  EXPECT_EQ(corrsAngular.size(), 1u);
}

TEST(CSimplePointsMapTests, determineMatching3DFindsCorrespondences)
{
  const auto globalMap = load_demo_9pts_map<CSimplePointsMap>();
  const auto otherMap = load_demo_9pts_map<CSimplePointsMap>();

  mrpt::maps::TMatchingParams params;
  params.maxDistForCorrespondence = 0.1f;
  mrpt::maps::TMatchingExtraResults extraResults;
  mrpt::tfest::TMatchingPairList correspondences;

  globalMap.determineMatching3D(
      &otherMap, CPose3D::Identity(), correspondences, params, extraResults);

  EXPECT_EQ(correspondences.size(), demo9_N);
  EXPECT_FLOAT_EQ(extraResults.correspondencesRatio, 1.0f);
}

TEST(CSimplePointsMapTests, determineMatching3DEmptyGlobalMapReturnsNoCorrespondences)
{
  CSimplePointsMap emptyMap;
  const auto otherMap = load_demo_9pts_map<CSimplePointsMap>();

  mrpt::maps::TMatchingParams params;
  mrpt::maps::TMatchingExtraResults extraResults;
  mrpt::tfest::TMatchingPairList correspondences;

  emptyMap.determineMatching3D(
      &otherMap, CPose3D::Identity(), correspondences, params, extraResults);

  EXPECT_TRUE(correspondences.empty());
}

TEST(CSimplePointsMapTests, compute3DMatchingRatioIdenticalClouds)
{
  const auto globalMap = load_demo_9pts_map<CSimplePointsMap>();
  const auto otherMap = load_demo_9pts_map<CSimplePointsMap>();

  mrpt::maps::TMatchingRatioParams mrp;
  mrp.maxDistForCorr = 0.1f;

  const float ratio = globalMap.compute3DMatchingRatio(&otherMap, CPose3D::Identity(), mrp);
  EXPECT_FLOAT_EQ(ratio, 1.0f);
}

// ----------------------------------------------------------------------
// Tests for boundingBox().
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, boundingBoxEmptyMap)
{
  const CSimplePointsMap emptyMap;
  const auto bb = emptyMap.boundingBox();
  EXPECT_FLOAT_EQ(bb.min.x, 0.0f);
  EXPECT_FLOAT_EQ(bb.min.y, 0.0f);
  EXPECT_FLOAT_EQ(bb.min.z, 0.0f);
  EXPECT_FLOAT_EQ(bb.max.x, 0.0f);
  EXPECT_FLOAT_EQ(bb.max.y, 0.0f);
  EXPECT_FLOAT_EQ(bb.max.z, 0.0f);
}

TEST(CSimplePointsMapTests, boundingBoxNonEmptyMap)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();
  const auto bb = pts.boundingBox();
  EXPECT_FLOAT_EQ(bb.min.x, 0.0f);
  EXPECT_FLOAT_EQ(bb.min.y, 0.0f);
  EXPECT_FLOAT_EQ(bb.min.z, 0.0f);
  EXPECT_FLOAT_EQ(bb.max.x, 2.0f);
  EXPECT_FLOAT_EQ(bb.max.y, 2.0f);
  EXPECT_FLOAT_EQ(bb.max.z, 2.0f);
}

// ----------------------------------------------------------------------
// Tests for extractCylinder() and extractPoints().
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, extractCylinderKeepsPointsInside)
{
  auto pts = load_demo_9pts_map<CSimplePointsMap>();

  CSimplePointsMap outMap;
  pts.extractCylinder(TPoint2D(1, 1), 0.5, -10, 10, outMap);

  // Only the point at (1,1,1) lies within a radius of 0.5 from (1,1):
  ASSERT_EQ(outMap.size(), 1u);
  float x, y, z;
  outMap.getPoint(0, x, y, z);
  EXPECT_FLOAT_EQ(x, 1.0f);
  EXPECT_FLOAT_EQ(y, 1.0f);
  EXPECT_FLOAT_EQ(z, 1.0f);
}

TEST(CSimplePointsMapTests, extractPointsKeepsPointsInsideBoundingBox)
{
  auto pts = load_demo_9pts_map<CSimplePointsMap>();

  const TBoundingBoxf bbox({-0.5f, -0.5f, -0.5f}, {1.5f, 1.5f, 1.5f});
  CSimplePointsMap outMap;
  pts.extractPoints(bbox, outMap);

  // Points (0,0,0) (0,1,1) (1,0,0) (1,1,1) fall inside the box:
  EXPECT_EQ(outMap.size(), 4u);
}

// ----------------------------------------------------------------------
// Test for compute3DDistanceToMesh().
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, compute3DDistanceToMeshSmoke)
{
  CSimplePointsMap globalMap;
  globalMap.insertPoint(0.0f, 0.0f, 0.0f);
  globalMap.insertPoint(1.0f, 0.0f, 0.0f);
  globalMap.insertPoint(0.0f, 1.0f, 0.0f);
  globalMap.insertPoint(1.0f, 1.0f, 0.0f);

  CSimplePointsMap otherMap;
  otherMap.insertPoint(0.33f, 0.33f, 0.0f);

  mrpt::tfest::TMatchingPairList correspondences;
  float correspondencesRatio = 0;

  globalMap.compute3DDistanceToMesh(
      &otherMap, CPose3D::Identity(), 1.0f, correspondences, correspondencesRatio);

  EXPECT_GT(correspondences.size(), 0u);
  EXPECT_GT(correspondencesRatio, 0.0f);
}

// ----------------------------------------------------------------------
// Tests for internal_computeObservationLikelihood() and insertObservation()
// branching on the runtime type of the input CObservation.
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, computeObservationLikelihood2DScanHorizontal)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  CSimplePointsMap globalMap;
  globalMap.insertObservation(scan1);

  const double ll = globalMap.computeObservationLikelihood(scan1, CPose3D::Identity());
  EXPECT_TRUE(std::isfinite(ll));
  EXPECT_LE(ll, 0.0);
}

TEST(CSimplePointsMapTests, computeObservationLikelihood2DScanNonHorizontal)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  CSimplePointsMap globalMap;
  globalMap.insertObservation(scan1);

  // A non-horizontal "takenFrom" pose forces the generic 3D likelihood branch:
  const CPose3D takenFrom(0, 0, 0, 0.0_deg, 20.0_deg, 0.0_deg);
  const double ll = globalMap.computeObservationLikelihood(scan1, takenFrom);
  EXPECT_TRUE(std::isfinite(ll));
}

TEST(CSimplePointsMapTests, computeObservationLikelihoodVelodyneScan)
{
  CSimplePointsMap globalMap = load_demo_9pts_map<CSimplePointsMap>();

  mrpt::obs::CObservationVelodyneScan scan;
  scan.point_cloud.x = {0.0f, 1.0f, 2.0f};
  scan.point_cloud.y = {0.0f, 0.0f, 0.0f};
  scan.point_cloud.z = {0.0f, 0.0f, 0.0f};
  scan.point_cloud.intensity = {0, 0, 0};

  const double ll = globalMap.computeObservationLikelihood(scan, CPose3D::Identity());
  EXPECT_TRUE(std::isfinite(ll));
  EXPECT_NE(ll, -100.0);
}

TEST(CSimplePointsMapTests, computeObservationLikelihoodPointCloud)
{
  CSimplePointsMap globalMap = load_demo_9pts_map<CSimplePointsMap>();

  auto localCloud = CSimplePointsMap::Create();
  localCloud->insertPoint(0.0f, 0.0f, 0.0f);
  localCloud->insertPoint(1.0f, 1.0f, 1.0f);

  mrpt::obs::CObservationPointCloud obs;
  obs.pointcloud = localCloud;
  obs.sensorPose = CPose3D::Identity();

  const double ll = globalMap.computeObservationLikelihood(obs, CPose3D::Identity());
  EXPECT_TRUE(std::isfinite(ll));
  EXPECT_NE(ll, -100.0);
}

TEST(CSimplePointsMapTests, computeObservationLikelihoodUnhandledObservationReturnsZero)
{
  CSimplePointsMap globalMap = load_demo_9pts_map<CSimplePointsMap>();

  mrpt::obs::CObservationComment obs;
  const double ll = globalMap.computeObservationLikelihood(obs, CPose3D::Identity());
  EXPECT_DOUBLE_EQ(ll, 0.0);
}

TEST(CSimplePointsMapTests, computeObservationLikelihoodEmptyGlobalMapReturnsSentinel)
{
  CSimplePointsMap emptyMap;

  mrpt::obs::CObservationVelodyneScan scan;
  scan.point_cloud.x = {0.0f};
  scan.point_cloud.y = {0.0f};
  scan.point_cloud.z = {0.0f};
  scan.point_cloud.intensity = {0};

  const double ll = emptyMap.computeObservationLikelihood(scan, CPose3D::Identity());
  EXPECT_DOUBLE_EQ(ll, -100.0);
}

TEST(CSimplePointsMapTests, insertObservationUnhandledTypeReturnsFalse)
{
  CSimplePointsMap pts;
  mrpt::obs::CObservationComment obs;
  EXPECT_FALSE(pts.insertObservation(obs));
  EXPECT_EQ(pts.size(), 0u);
}

// ----------------------------------------------------------------------
// Tests for insertAnotherMap() and fuseWith().
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, insertAnotherMapAppliesPoseTransform)
{
  const auto src = load_demo_9pts_map<CSimplePointsMap>();

  CSimplePointsMap dst;
  const CPose3D pose(1.0, 2.0, 0.0, 0.0, 0.0, 0.0);
  dst.insertAnotherMap(&src, pose);

  ASSERT_EQ(dst.size(), demo9_N);
  float x, y, z;
  dst.getPoint(0, x, y, z);
  EXPECT_NEAR(x, demo9_xs[0] + 1.0f, 1e-5f);
  EXPECT_NEAR(y, demo9_ys[0] + 2.0f, 1e-5f);
  EXPECT_NEAR(z, demo9_zs[0], 1e-5f);
}

TEST(CSimplePointsMapTests, fuseWithMergesCloseAndAddsNewPoints)
{
  CSimplePointsMap mapA;
  mapA.insertPoint(0.0f, 0.0f, 0.0f);
  mapA.insertPoint(5.0f, 5.0f, 5.0f);

  CSimplePointsMap mapB;
  mapB.insertPoint(0.01f, 0.0f, 0.0f);    // Close to mapA's first point: gets fused.
  mapB.insertPoint(10.0f, 10.0f, 10.0f);  // Far from any point: gets added.

  std::vector<bool> notFused;
  mapA.fuseWith(&mapB, 0.5f, &notFused);

  EXPECT_EQ(mapA.size(), 3u);
  ASSERT_EQ(notFused.size(), 3u);
  // The second original point (5,5,5) was never touched by fusion:
  EXPECT_TRUE(notFused[1]);
}

// ----------------------------------------------------------------------
// Tests for loadFromKittiVelodyneFile() / saveToKittiVelodyneFile().
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, loadFromKittiVelodyneFileNonexistentReturnsFalse)
{
  CSimplePointsMap pts;
  EXPECT_FALSE(pts.loadFromKittiVelodyneFile("/nonexistent_dir_xyz_abc/scan.bin"));
}

TEST(CGenericPointsMapTests, loadFromKittiVelodyneFileTruncatedRecordReturnsFalse)
{
  const std::string file = makeTempFilePath("_truncated.bin");
  {
    std::ofstream ofs(file, std::ios::binary);
    const char garbage[5] = {1, 2, 3, 4, 5};
    ofs.write(garbage, sizeof(garbage));
  }

  CGenericPointsMap pts;
  EXPECT_FALSE(pts.loadFromKittiVelodyneFile(file));

  std::filesystem::remove(file);
}

TEST(CGenericPointsMapTests, kittiVelodyneRoundTrip)
{
  // saveToKittiVelodyneFile() always writes gzip-compressed data, and
  // loadFromKittiVelodyneFile() only auto-detects gzip via the ".gz"
  // extension, so the round trip requires that extension.
  CGenericPointsMap pts;
  pts.registerField_float(CPointsMap::POINT_FIELD_INTENSITY);
  pts.insertPointFast(1.0f, 2.0f, 3.0f);
  pts.insertPointField_float(CPointsMap::POINT_FIELD_INTENSITY, 0.5f);
  pts.insertPointFast(-1.0f, -2.0f, -3.0f);
  pts.insertPointField_float(CPointsMap::POINT_FIELD_INTENSITY, 0.75f);

  const std::string file = makeTempFilePath(".gz");
  ASSERT_TRUE(pts.saveToKittiVelodyneFile(file));

  CGenericPointsMap pts2;
  ASSERT_TRUE(pts2.loadFromKittiVelodyneFile(file));

  ASSERT_EQ(pts2.size(), pts.size());
  const auto* intensities = pts2.getPointsBufferRef_float_field(CPointsMap::POINT_FIELD_INTENSITY);
  ASSERT_TRUE(intensities != nullptr);
  EXPECT_FLOAT_EQ(intensities->at(0), 0.5f);
  EXPECT_FLOAT_EQ(intensities->at(1), 0.75f);

  float x, y, z;
  pts2.getPoint(0, x, y, z);
  EXPECT_FLOAT_EQ(x, 1.0f);
  EXPECT_FLOAT_EQ(y, 2.0f);
  EXPECT_FLOAT_EQ(z, 3.0f);

  std::filesystem::remove(file);
}

// ----------------------------------------------------------------------
// Test for PLY import/export round trip through the generic
// PLY_Importer/PLY_Exporter machinery.
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, plySaveLoadRoundTrip)
{
  const auto pts0 = load_demo_9pts_map<CSimplePointsMap>();
  const std::string file = makeTempFilePath(".ply");

  ASSERT_TRUE(pts0.saveToPlyFile(file));

  CSimplePointsMap pts1;
  ASSERT_TRUE(pts1.loadFromPlyFile(file));

  ASSERT_EQ(pts1.size(), pts0.size());
  for (size_t i = 0; i < pts1.size(); i++)
  {
    float x, y, z;
    pts1.getPoint(i, x, y, z);
    EXPECT_NEAR(x, demo9_xs[i], 1e-4f);
    EXPECT_NEAR(y, demo9_ys[i], 1e-4f);
    EXPECT_NEAR(z, demo9_zs[i], 1e-4f);
  }

  std::filesystem::remove(file);
}

// ----------------------------------------------------------------------
// Tests for miscellaneous small CPointsMap utility methods.
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, getPointFieldNamesFloatExceptXyz)
{
  const CSimplePointsMap pts;
  const auto names = pts.getPointFieldNames_float_except_xyz();
  EXPECT_TRUE(names.empty());
}

TEST(CGenericPointsMapTests, getPointFieldNamesFloatExceptXyz)
{
  CGenericPointsMap pts;
  pts.registerField_float("intensity");
  const auto names = pts.getPointFieldNames_float_except_xyz();
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "intensity");
}

TEST(CGenericPointsMapTests, hasColorU8AndF)
{
  CGenericPointsMap pts;
  EXPECT_FALSE(pts.hasColor_u8());
  EXPECT_FALSE(pts.hasColor_f());

  pts.registerField_uint8(CPointsMap::POINT_FIELD_COLOR_Ru8);
  pts.registerField_uint8(CPointsMap::POINT_FIELD_COLOR_Gu8);
  pts.registerField_uint8(CPointsMap::POINT_FIELD_COLOR_Bu8);
  EXPECT_TRUE(pts.hasColor_u8());
  EXPECT_FALSE(pts.hasColor_f());

  pts.registerField_float(CPointsMap::POINT_FIELD_COLOR_Rf);
  pts.registerField_float(CPointsMap::POINT_FIELD_COLOR_Gf);
  pts.registerField_float(CPointsMap::POINT_FIELD_COLOR_Bf);
  EXPECT_TRUE(pts.hasColor_f());
}

TEST(CSimplePointsMapTests, asStringContainsSizeAndFields)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();
  const std::string s = pts.asString();
  EXPECT_NE(s.find("x,y,z"), std::string::npos);
}

TEST(CSimplePointsMapTests, optionsByNameReturnsAllThree)
{
  CSimplePointsMap pts;
  const auto opts = pts.optionsByName();
  EXPECT_EQ(opts.size(), 3u);
  EXPECT_NE(opts.find("insertionOptions"), opts.end());
  EXPECT_NE(opts.find("likelihoodOptions"), opts.end());
  EXPECT_NE(opts.find("renderOptions"), opts.end());
}

TEST(CSimplePointsMapTests, nnPrepareForQueriesDoesNotThrow)
{
  const auto pts = load_demo_9pts_map<CSimplePointsMap>();
  EXPECT_NO_THROW(pts.nn_prepare_for_2d_queries());
  EXPECT_NO_THROW(pts.nn_prepare_for_3d_queries());
  EXPECT_TRUE(pts.nn_has_indices_or_ids());
  EXPECT_EQ(pts.nn_index_count(), demo9_N);
}

// ----------------------------------------------------------------------
// Tests for filter-by-height and also_interpolate insertion behavior,
// exercised through insertObservation() of a 2D range scan.
// ----------------------------------------------------------------------

TEST(CSimplePointsMapTests, heightFilterExcludesOutOfRangePoints)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  CSimplePointsMap pnt;
  EXPECT_FALSE(pnt.isFilterByHeightEnabled());

  pnt.enableFilterByHeight(true);
  EXPECT_TRUE(pnt.isFilterByHeightEnabled());

  pnt.setHeightFilterLevels(5.0, 10.0);
  const auto levels = pnt.getHeightFilterLevels();
  EXPECT_DOUBLE_EQ(levels.first, 5.0);
  EXPECT_DOUBLE_EQ(levels.second, 10.0);

  pnt.insertObservation(scan1);

  // All points of this planar scan lie at z=0, outside the [5,10] range:
  EXPECT_EQ(pnt.size(), 0u);
}

TEST(CSimplePointsMapTests, insert2DScanWithInterpolation)
{
  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  CSimplePointsMap pnt;
  pnt.insertionOptions.also_interpolate = true;
  pnt.insertionOptions.minDistBetweenLaserPoints = 0.01f;
  pnt.insertionOptions.maxDistForInterpolatePoints = 5.0f;

  pnt.insertObservation(scan1);

  // Interpolation may add extra points between consecutive scan readings:
  EXPECT_GE(pnt.size(), 267u);
}
