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
#include <mrpt/math/KDTreeCapable.h>

#include <cmath>
#include <nanoflann.hpp>
#include <vector>

/** Minimal 2D point cloud adapter for KDTreeCapable. */
struct PointCloud2D : public mrpt::math::KDTreeCapable<PointCloud2D>
{
  std::vector<mrpt::math::TPoint2D> pts;

  size_t kdtree_get_point_count() const { return pts.size(); }
  float kdtree_get_pt(size_t idx, int dim) const
  {
    return dim == 0 ? mrpt::d2f(pts[idx].x) : mrpt::d2f(pts[idx].y);
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const
  {
    return false;
  }
};

/** Minimal 3D point cloud adapter for KDTreeCapable. */
struct PointCloud3D : public mrpt::math::KDTreeCapable<PointCloud3D>
{
  std::vector<mrpt::math::TPoint3D> pts;

  size_t kdtree_get_point_count() const { return pts.size(); }
  float kdtree_get_pt(size_t idx, int dim) const
  {
    if (dim == 0) return mrpt::d2f(pts[idx].x);
    if (dim == 1) return mrpt::d2f(pts[idx].y);
    return mrpt::d2f(pts[idx].z);
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const
  {
    return false;
  }
};

TEST(KDTreeCapable, closestPoint2D)
{
  PointCloud2D cloud;
  cloud.pts = {
      { 0,  0},
      { 1,  0},
      { 0,  1},
      { 2,  2},
      {-1, -1}
  };

  float cx, cy, distSqr;
  const size_t idx = cloud.kdTreeClosestPoint2D(0.1f, 0.05f, cx, cy, distSqr);

  EXPECT_EQ(idx, 0u);
  EXPECT_NEAR(cx, 0.f, 1e-5f);
  EXPECT_NEAR(cy, 0.f, 1e-5f);
  EXPECT_LT(distSqr, 0.02f);
}

TEST(KDTreeCapable, closestPoint2D_nearBoundary)
{
  PointCloud2D cloud;
  cloud.pts = {
      {0, 0},
      {3, 0},
      {3, 3}
  };

  float cx, cy, distSqr;
  const size_t idx = cloud.kdTreeClosestPoint2D(2.9f, 0.1f, cx, cy, distSqr);

  EXPECT_EQ(idx, 1u);
  EXPECT_NEAR(cx, 3.f, 1e-4f);
}

TEST(KDTreeCapable, nClosestPoint2D)
{
  PointCloud2D cloud;
  cloud.pts = {
      { 0,  0},
      { 1,  0},
      { 0,  1},
      {10, 10}
  };

  std::vector<float> xs, ys, dists;
  const auto idxs = cloud.kdTreeNClosestPoint2D(0.f, 0.f, 3, xs, ys, dists);

  ASSERT_EQ(idxs.size(), 3u);
  // The 3 closest should NOT include (10,10):
  for (size_t i = 0; i < 3; i++) EXPECT_LT(dists[i], 5.f);
  // First result should be (0,0) itself:
  EXPECT_EQ(idxs[0], 0u);
}

TEST(KDTreeCapable, closestPoint3D)
{
  PointCloud3D cloud;
  cloud.pts = {
      {0, 0, 0},
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, 1}
  };

  float ox, oy, oz, distSqr;
  const size_t idx = cloud.kdTreeClosestPoint3D(0.1f, 0.1f, 0.8f, ox, oy, oz, distSqr);

  EXPECT_EQ(idx, 3u);
  EXPECT_NEAR(oz, 1.f, 1e-4f);
}

TEST(KDTreeCapable, radiusSearch2D)
{
  PointCloud2D cloud;
  cloud.pts = {
      {0, 0},
      {1, 0},
      {0, 1},
      {5, 5}
  };

  std::vector<nanoflann::ResultItem<size_t, float>> results;
  const size_t count = cloud.kdTreeRadiusSearch2D(0.f, 0.f, 1.5f, results);

  // Should find (0,0), (1,0), (0,1) — all within radius 1.5:
  EXPECT_EQ(count, 3u);
  EXPECT_EQ(results.size(), 3u);
  for (const auto& r : results)
    EXPECT_LT(r.first, 3u);  // index should be 0, 1, or 2 (not the far point)
}
