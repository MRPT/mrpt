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
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>

#include <nanoflann.hpp>
#include <vector>

namespace
{
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
  bool kdtree_get_bbox([[maybe_unused]] BBOX& bbox) const
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
    if (dim == 0)
    {
      return mrpt::d2f(pts[idx].x);
    }
    if (dim == 1)
    {
      return mrpt::d2f(pts[idx].y);
    }
    return mrpt::d2f(pts[idx].z);
  }
  template <class BBOX>
  bool kdtree_get_bbox([[maybe_unused]] BBOX& bbox) const
  {
    return false;
  }
};
}  // namespace

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

  mrpt::math::TPoint2Df c;
  float distSqr = 0;
  const size_t idx = cloud.kdTreeClosestPoint2D(0.1f, 0.05f, c.x, c.y, distSqr);

  EXPECT_EQ(idx, 0u);
  EXPECT_NEAR(c.x, 0.f, 1e-5f);
  EXPECT_NEAR(c.y, 0.f, 1e-5f);
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

  mrpt::math::TPoint2Df c;
  float distSqr = 0;
  const size_t idx = cloud.kdTreeClosestPoint2D(2.9f, 0.1f, c.x, c.y, distSqr);

  EXPECT_EQ(idx, 1u);
  EXPECT_NEAR(c.x, 3.f, 1e-4f);
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

  std::vector<float> xs;
  std::vector<float> ys;
  std::vector<float> dists;
  const auto idxs = cloud.kdTreeNClosestPoint2D(0.f, 0.f, 3, xs, ys, dists);

  ASSERT_EQ(idxs.size(), 3u);
  // The 3 closest should NOT include (10,10):
  for (size_t i = 0; i < 3; i++)
  {
    EXPECT_LT(dists[i], 5.f);
  }
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

  mrpt::math::TPoint3Df o;
  float distSqr = 0;
  const size_t idx = cloud.kdTreeClosestPoint3D(0.1f, 0.1f, 0.8f, o.x, o.y, o.z, distSqr);

  EXPECT_EQ(idx, 3u);
  EXPECT_NEAR(o.z, 1.f, 1e-4f);
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
  {
    EXPECT_LT(r.first, 3u);  // index should be 0, 1, or 2 (not the far point)
  }
}

namespace
{
PointCloud2D makeGrid2D()
{
  PointCloud2D cloud;
  for (int i = 0; i < 5; i++)
    for (int j = 0; j < 5; j++) cloud.pts.emplace_back(i, j);
  return cloud;
}

PointCloud3D makeGrid3D()
{
  PointCloud3D cloud;
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      for (int k = 0; k < 4; k++) cloud.pts.emplace_back(i, j, k);
  return cloud;
}
}  // namespace

TEST(KDTreeCapable, closestPoint2DOverloads)
{
  auto cloud = makeGrid2D();

  float distSqr = 0;
  const size_t idx = cloud.kdTreeClosestPoint2D(1.1f, 2.1f, distSqr);
  EXPECT_EQ(cloud.pts[idx], mrpt::math::TPoint2D(1, 2));

  mrpt::math::TPoint2D pOut;
  const size_t idx2 = cloud.kdTreeClosestPoint2D({1.1, 2.1}, pOut, distSqr);
  EXPECT_EQ(idx2, idx);
  EXPECT_NEAR(pOut.x, 1.0, 1e-4);

  EXPECT_NEAR(cloud.kdTreeClosestPoint2DsqrError(1.0f, 2.0f), 0.0, 1e-6);
  EXPECT_NEAR(cloud.kdTreeClosestPoint2DsqrError(mrpt::math::TPoint2D(1.0, 2.0)), 0.0, 1e-6);

  PointCloud2D empty;
  EXPECT_THROW(empty.kdTreeClosestPoint2D(0.f, 0.f, distSqr), std::exception);
}

TEST(KDTreeCapable, twoClosestPoint2D)
{
  auto cloud = makeGrid2D();

  float x1 = 0, y1 = 0, x2 = 0, y2 = 0, d1 = 0, d2 = 0;
  cloud.kdTreeTwoClosestPoint2D(0.1f, 0.0f, x1, y1, x2, y2, d1, d2);
  EXPECT_NEAR(x1, 0.0f, 1e-4f);
  EXPECT_NEAR(y1, 0.0f, 1e-4f);
  EXPECT_LE(d1, d2);

  mrpt::math::TPoint2D p1, p2;
  cloud.kdTreeTwoClosestPoint2D({0.1, 0.0}, p1, p2, d1, d2);
  EXPECT_NEAR(p1.x, 0.0, 1e-4);
  EXPECT_LE(d1, d2);
}

TEST(KDTreeCapable, nClosestPoint2DWithMaxDistance)
{
  auto cloud = makeGrid2D();

  std::vector<size_t> idxs;
  std::vector<float> dists;

  // Without a radius limit, exactly `knn` neighbors come back:
  cloud.kdTreeNClosestPoint2DIdx(0.f, 0.f, 5, idxs, dists);
  EXPECT_EQ(idxs.size(), 5U);

  // With a radius smaller than the grid step, only the query point itself:
  cloud.kdTreeNClosestPoint2DIdx(0.f, 0.f, 5, idxs, dists, 0.5f);
  EXPECT_EQ(idxs.size(), 1U);
  EXPECT_EQ(dists.size(), 1U);

  // The same, through the TPoint2D overload:
  cloud.kdTreeNClosestPoint2DIdx(mrpt::math::TPoint2D(0, 0), 5, idxs, dists);
  EXPECT_EQ(idxs.size(), 5U);

  std::vector<float> xs, ys;
  auto found = cloud.kdTreeNClosestPoint2D(0.f, 0.f, 5, xs, ys, dists, 0.5f);
  EXPECT_EQ(found.size(), 1U);
  // All the output vectors must be trimmed to the number of points found:
  EXPECT_EQ(xs.size(), 1U);
  EXPECT_EQ(ys.size(), 1U);
  EXPECT_EQ(dists.size(), 1U);

  std::vector<mrpt::math::TPoint2D> limitedPts;
  found = cloud.kdTreeNClosestPoint2D(mrpt::math::TPoint2D(0, 0), 5, limitedPts, dists, 0.5f);
  EXPECT_EQ(limitedPts.size(), 1U);

  std::vector<mrpt::math::TPoint2D> pts;
  found = cloud.kdTreeNClosestPoint2D(mrpt::math::TPoint2D(0, 0), 3, pts, dists);
  EXPECT_EQ(found.size(), 3U);
  EXPECT_EQ(pts.size(), 3U);
}

TEST(KDTreeCapable, closestPoint3DOverloads)
{
  auto cloud = makeGrid3D();

  float distSqr = 0;
  const size_t idx = cloud.kdTreeClosestPoint3D(1.1f, 2.1f, 3.1f, distSqr);
  EXPECT_EQ(cloud.pts[idx], mrpt::math::TPoint3D(1, 2, 3));

  mrpt::math::TPoint3D pOut;
  const size_t idx2 = cloud.kdTreeClosestPoint3D({1.1, 2.1, 3.1}, pOut, distSqr);
  EXPECT_EQ(idx2, idx);
  EXPECT_NEAR(pOut.z, 3.0, 1e-4);

  PointCloud3D empty;
  EXPECT_THROW(empty.kdTreeClosestPoint3D(0.f, 0.f, 0.f, distSqr), std::exception);
}

TEST(KDTreeCapable, nClosestPoint3DWithMaxDistance)
{
  auto cloud = makeGrid3D();

  std::vector<size_t> idxs;
  std::vector<float> dists;

  cloud.kdTreeNClosestPoint3DIdx(0.f, 0.f, 0.f, 4, idxs, dists);
  EXPECT_EQ(idxs.size(), 4U);

  cloud.kdTreeNClosestPoint3DIdx(0.f, 0.f, 0.f, 4, idxs, dists, 0.5f);
  EXPECT_EQ(idxs.size(), 1U);

  cloud.kdTreeNClosestPoint3DIdx(mrpt::math::TPoint3D(0, 0, 0), 4, idxs, dists);
  EXPECT_EQ(idxs.size(), 4U);

  std::vector<float> xs, ys, zs;
  cloud.kdTreeNClosestPoint3D(0.f, 0.f, 0.f, 4, xs, ys, zs, dists, 0.5f);
  EXPECT_EQ(xs.size(), 1U);
  EXPECT_EQ(ys.size(), 1U);
  EXPECT_EQ(zs.size(), 1U);

  std::vector<size_t> limitedIdx;
  cloud.kdTreeNClosestPoint3DWithIdx(0.f, 0.f, 0.f, 4, xs, ys, zs, limitedIdx, dists, 0.5f);
  EXPECT_EQ(xs.size(), 1U);
  EXPECT_EQ(limitedIdx.size(), 1U);

  std::vector<mrpt::math::TPoint3D> pts;
  cloud.kdTreeNClosestPoint3D(mrpt::math::TPoint3D(0, 0, 0), 3, pts, dists);
  EXPECT_EQ(pts.size(), 3U);
}

TEST(KDTreeCapable, radiusSearch3DAndEmptyClouds)
{
  auto cloud = makeGrid3D();

  std::vector<nanoflann::ResultItem<size_t, float>> results;
  const size_t n = cloud.kdTreeRadiusSearch3D(0.f, 0.f, 0.f, 1.1f, results);
  EXPECT_EQ(n, results.size());
  EXPECT_GE(n, 4U);  // itself plus the 3 axis neighbors at distance 1

  // Radius searches on an empty cloud return zero results instead of throwing:
  PointCloud3D empty3d;
  EXPECT_EQ(empty3d.kdTreeRadiusSearch3D(0.f, 0.f, 0.f, 10.f, results), 0U);

  PointCloud2D empty2d;
  std::vector<nanoflann::ResultItem<size_t, float>> results2d;
  EXPECT_EQ(empty2d.kdTreeRadiusSearch2D(0.f, 0.f, 10.f, results2d), 0U);
}

// Exposes the protected invalidation hook, as any real map class does when
// its contents change.
namespace
{
struct MutablePointCloud2D : public PointCloud2D
{
  void markOutdated() { this->kdtree_mark_as_outdated(); }
};
}  // namespace

TEST(KDTreeCapable, treeIsRebuiltWhenCloudChanges)
{
  MutablePointCloud2D cloud;
  cloud.pts.emplace_back(0, 0);

  float distSqr = 0;
  EXPECT_EQ(cloud.kdTreeClosestPoint2D(5.f, 5.f, distSqr), 0U);

  cloud.pts.emplace_back(5, 5);
  cloud.markOutdated();
  EXPECT_EQ(cloud.kdTreeClosestPoint2D(5.f, 5.f, distSqr), 1U);
  EXPECT_NEAR(distSqr, 0.0f, 1e-6f);
}

TEST(KDTreeCapable, kNNRequestLargerThanCloud)
{
  // Asking for more neighbors than the cloud holds must return only as many
  // as exist, in *every* output vector, rather than leaving stale entries.
  PointCloud2D cloud2d;
  cloud2d.pts.emplace_back(0, 0);
  cloud2d.pts.emplace_back(1, 0);
  cloud2d.pts.emplace_back(0, 1);

  std::vector<size_t> idxs;
  std::vector<float> dists, xs, ys, zs;

  cloud2d.kdTreeNClosestPoint2DIdx(0.f, 0.f, 10, idxs, dists);
  EXPECT_EQ(idxs.size(), 3U);
  EXPECT_EQ(dists.size(), 3U);

  auto found = cloud2d.kdTreeNClosestPoint2D(0.f, 0.f, 10, xs, ys, dists);
  EXPECT_EQ(found.size(), 3U);
  EXPECT_EQ(xs.size(), 3U);
  EXPECT_EQ(ys.size(), 3U);
  EXPECT_EQ(dists.size(), 3U);

  std::vector<mrpt::math::TPoint2D> pts2d;
  found = cloud2d.kdTreeNClosestPoint2D(mrpt::math::TPoint2D(0, 0), 10, pts2d, dists);
  EXPECT_EQ(pts2d.size(), 3U);
  EXPECT_EQ(pts2d.size(), dists.size());

  PointCloud3D cloud3d;
  cloud3d.pts.emplace_back(0, 0, 0);
  cloud3d.pts.emplace_back(1, 0, 0);

  cloud3d.kdTreeNClosestPoint3DIdx(0.f, 0.f, 0.f, 10, idxs, dists);
  EXPECT_EQ(idxs.size(), 2U);
  EXPECT_EQ(dists.size(), 2U);

  cloud3d.kdTreeNClosestPoint3D(0.f, 0.f, 0.f, 10, xs, ys, zs, dists);
  EXPECT_EQ(xs.size(), 2U);
  EXPECT_EQ(ys.size(), 2U);
  EXPECT_EQ(zs.size(), 2U);
  EXPECT_EQ(dists.size(), 2U);

  cloud3d.kdTreeNClosestPoint3DWithIdx(0.f, 0.f, 0.f, 10, xs, ys, zs, idxs, dists);
  EXPECT_EQ(xs.size(), 2U);
  EXPECT_EQ(ys.size(), 2U);
  EXPECT_EQ(zs.size(), 2U);
  EXPECT_EQ(idxs.size(), 2U);
  EXPECT_EQ(dists.size(), 2U);

  std::vector<mrpt::math::TPoint3D> pts3d;
  cloud3d.kdTreeNClosestPoint3D(mrpt::math::TPoint3D(0, 0, 0), 10, pts3d, dists);
  EXPECT_EQ(pts3d.size(), 2U);
  EXPECT_EQ(pts3d.size(), dists.size());
}
