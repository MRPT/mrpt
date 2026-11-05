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
#include <mrpt/core/bits_math.h>
#include <mrpt/math/KDTreeCapable.h>
#include <mrpt/math/TPoint2D.h>
#include <mrpt/math/TPoint3D.h>
#include <mrpt/random/RandomGenerators.h>

#include <algorithm>
#include <array>
#include <nanoflann.hpp>
#include <sstream>
#include <stdexcept>
#include <utility>
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

/** A cloud that maintains its own index and therefore opts out of the cached
 *  one offered by the base class. \sa KDTreeCapable::kdtree_disable() */
struct PointCloud3DNoKDTree : public PointCloud3D
{
  PointCloud3DNoKDTree() { kdtree_disable("PointCloud3DNoKDTree has its own index."); }

  /// kdtree_is_enabled() is protected; expose it for the test below.
  [[nodiscard]] bool isKdTreeEnabled() const { return kdtree_is_enabled(); }
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

TEST(KDTreeCapable, repeatedMarksStillForceRebuild)
{
  MutablePointCloud2D cloud;
  cloud.pts.emplace_back(0, 0);

  float distSqr = 0;
  EXPECT_EQ(cloud.kdTreeClosestPoint2D(5.f, 5.f, distSqr), 0U);

  // Marking an already outdated index must keep it outdated:
  for (int i = 0; i < 3; i++)
  {
    cloud.pts.emplace_back(10 + i, 10 + i);
    cloud.markOutdated();
  }
  cloud.pts.emplace_back(5, 5);
  cloud.markOutdated();
  EXPECT_EQ(cloud.kdTreeClosestPoint2D(5.f, 5.f, distSqr), 4U);
  EXPECT_NEAR(distSqr, 0.0f, 1e-6f);

  // ...and the next change after a rebuild must be noticed too:
  cloud.pts.emplace_back(7, 7);
  cloud.markOutdated();
  EXPECT_EQ(cloud.kdTreeClosestPoint2D(7.f, 7.f, distSqr), 5U);
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

TEST(KDTreeCapable, disabledIndexThrows)
{
  PointCloud3DNoKDTree cloud;
  cloud.pts.emplace_back(0, 0, 0);
  cloud.pts.emplace_back(1, 0, 0);

  EXPECT_FALSE(cloud.isKdTreeEnabled());

  float distSqr = 0;
  float cx = 0;
  float cy = 0;
  float cz = 0;
  EXPECT_THROW(cloud.kdTreeClosestPoint3D(0.f, 0.f, 0.f, cx, cy, cz, distSqr), std::logic_error);

  std::vector<size_t> idxs;
  std::vector<float> dists;
  EXPECT_THROW(cloud.kdTreeNClosestPoint3DIdx(0.f, 0.f, 0.f, 2, idxs, dists), std::logic_error);

  std::vector<nanoflann::ResultItem<size_t, float>> radiusHits;
  EXPECT_THROW(cloud.kdTreeRadiusSearch3D(0.f, 0.f, 0.f, 100.f, radiusHits), std::logic_error);

  EXPECT_THROW(cloud.kdTreeEnsureIndexBuilt3D(), std::logic_error);
  EXPECT_THROW(cloud.kdTreeEnsureIndexBuilt2D(), std::logic_error);

  // The opt-out is a property of the class, so it survives copying:
  PointCloud3DNoKDTree cloud2 = cloud;
  EXPECT_FALSE(cloud2.isKdTreeEnabled());
  EXPECT_THROW(cloud2.kdTreeEnsureIndexBuilt3D(), std::logic_error);

  // ...and assigning from an enabled object does not bring it back:
  PointCloud3D enabled;
  enabled.pts.emplace_back(0, 0, 0);
  cloud2.PointCloud3D::operator=(enabled);
  EXPECT_FALSE(cloud2.isKdTreeEnabled());
  EXPECT_THROW(cloud2.kdTreeEnsureIndexBuilt3D(), std::logic_error);

  // ...while plain clouds of the very same base class are unaffected:
  PointCloud3D plain;
  plain.pts.emplace_back(0, 0, 0);
  EXPECT_NO_THROW(plain.kdTreeEnsureIndexBuilt3D());
}

namespace
{
template <int DIM>
struct Cloud : public mrpt::math::KDTreeCapable<Cloud<DIM>>
{
  std::vector<std::array<float, 3>> pts;

  size_t kdtree_get_point_count() const { return pts.size(); }
  float kdtree_get_pt(size_t idx, int dim) const { return pts.at(idx)[static_cast<size_t>(dim)]; }
  template <class BBOX>
  bool kdtree_get_bbox([[maybe_unused]] BBOX& bb) const
  {
    return false;
  }
  void markChanged() { this->kdtree_mark_as_outdated(); }
};

template <int DIM>
Cloud<DIM> randomCloud(size_t n)
{
  auto& rng = mrpt::random::getRandomGenerator();
  Cloud<DIM> c;
  c.pts.resize(n);
  for (auto& p : c.pts)
  {
    for (int d = 0; d < 3; d++)
    {
      p[static_cast<size_t>(d)] = d < DIM ? static_cast<float>(rng.drawUniform(-10.0, 10.0)) : 0.f;
    }
  }
  return c;
}

template <int DIM>
std::vector<std::pair<float, size_t>> bruteForce(const Cloud<DIM>& c, const float* q)
{
  std::vector<std::pair<float, size_t>> d;
  for (size_t i = 0; i < c.pts.size(); i++)
  {
    float s = 0;
    for (int k = 0; k < DIM; k++)
    {
      s += mrpt::square(c.pts[i][static_cast<size_t>(k)] - q[k]);
    }
    d.emplace_back(s, i);
  }
  std::sort(d.begin(), d.end());
  return d;
}
}  // namespace

TEST(KDTreeBruteForce, knn2DAnd3DMatchBruteForce)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  auto c2 = randomCloud<2>(500);
  auto c3 = randomCloud<3>(500);
  auto& rng = mrpt::random::getRandomGenerator();
  for (int it = 0; it < 50; it++)
  {
    const float q[3] = {
        static_cast<float>(rng.drawUniform(-12.0, 12.0)),
        static_cast<float>(rng.drawUniform(-12.0, 12.0)),
        static_cast<float>(rng.drawUniform(-12.0, 12.0))};
    const size_t k = 7;
    {
      std::vector<size_t> idx;
      std::vector<float> d2;
      c2.kdTreeNClosestPoint2DIdx(q[0], q[1], k, idx, d2);
      const auto bf = bruteForce(c2, q);
      ASSERT_EQ(idx.size(), k);
      for (size_t i = 0; i < k; i++)
      {
        EXPECT_EQ(idx[i], bf[i].second);
        EXPECT_NEAR(d2[i], bf[i].first, 1e-3f);
      }
      float dist = 0;
      EXPECT_EQ(c2.kdTreeClosestPoint2D(q[0], q[1], dist), bf[0].second);
    }
    {
      std::vector<size_t> idx;
      std::vector<float> d2;
      c3.kdTreeNClosestPoint3DIdx(q[0], q[1], q[2], k, idx, d2);
      const auto bf = bruteForce(c3, q);
      ASSERT_EQ(idx.size(), k);
      for (size_t i = 0; i < k; i++)
      {
        EXPECT_EQ(idx[i], bf[i].second);
        EXPECT_NEAR(d2[i], bf[i].first, 1e-3f);
      }
    }
  }
}

TEST(KDTreeBruteForce, radiusSearchUsesSquaredRadius)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  auto c3 = randomCloud<3>(800);
  const float q[3] = {0.5f, -1.0f, 2.0f};
  const float radius = 3.0f;
  std::vector<nanoflann::ResultItem<size_t, float>> res;
  const size_t n = c3.kdTreeRadiusSearch3D(q[0], q[1], q[2], radius * radius, res);
  const auto bf = bruteForce(c3, q);
  size_t expected = 0;
  for (const auto& p : bf)
  {
    if (p.first < radius * radius)
    {
      expected++;
    }
  }
  EXPECT_EQ(n, expected);
  EXPECT_GT(n, 0U);
  for (const auto& r : res)
  {
    EXPECT_LT(r.second, radius * radius);
  }
  // 2D tree on the same object, then the 3D tree again: both must be valid
  std::vector<nanoflann::ResultItem<size_t, float>> res2;
  c3.kdTreeRadiusSearch2D(q[0], q[1], radius * radius, res2);
  const auto n3 = c3.kdTreeRadiusSearch3D(q[0], q[1], q[2], radius * radius, res);
  EXPECT_EQ(n3, expected);
  EXPECT_GE(res2.size(), n3);
}

TEST(KDTreeBruteForce, indexRebuiltAfterMarkOutdated)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  auto c2 = randomCloud<2>(100);
  float d = 0;
  const size_t i0 = c2.kdTreeClosestPoint2D(100.f, 100.f, d);
  (void)i0;
  c2.pts.push_back({99.f, 99.f, 0.f});
  c2.markChanged();
  EXPECT_EQ(c2.kdTreeClosestPoint2D(100.f, 100.f, d), c2.pts.size() - 1);
  EXPECT_NEAR(d, 2.0f, 1e-4f);
}

// The second squared distance must be that of the second point
TEST(KDTreeBruteForce, twoClosestPoint2DSecondDistance)
{
  Cloud<2> c;
  c.pts = {
      {0.f, 0.f, 0.f},
      {1.f, 0.f, 0.f},
      {5.f, 5.f, 0.f}
  };
  float x1 = 0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  float d1 = 0;
  float d2 = 0;
  c.kdTreeTwoClosestPoint2D(0.f, 0.f, x1, y1, x2, y2, d1, d2);
  EXPECT_EQ(x2, 1.0f);
  EXPECT_NEAR(d1, 0.0f, 1e-6f);
  EXPECT_NEAR(d2, 1.0f, 1e-6f);
}

TEST(KDTreeBruteForce, twoClosestPoint2DNeedsTwoPoints)
{
  Cloud<2> c;
  c.pts = {
      {0.f, 0.f, 0.f}
  };
  float x1 = 0;
  float y1 = 0;
  float x2 = 0;
  float y2 = 0;
  float d1 = 0;
  float d2 = 0;
  EXPECT_ANY_THROW(c.kdTreeTwoClosestPoint2D(0.f, 0.f, x1, y1, x2, y2, d1, d2));
}

TEST(KDTreeBruteForce, zeroNeighborsRequested)
{
  mrpt::random::getRandomGenerator().randomize(1234);
  auto c2 = randomCloud<2>(10);
  auto c3 = randomCloud<3>(10);
  std::vector<size_t> idx = {1};
  std::vector<float> d2 = {1.f};
  c2.kdTreeNClosestPoint2DIdx(0.f, 0.f, 0, idx, d2);
  EXPECT_TRUE(idx.empty());
  EXPECT_TRUE(d2.empty());
  std::vector<float> xs = {1.f};
  std::vector<float> ys = {1.f};
  std::vector<float> zs = {1.f};
  EXPECT_TRUE(c2.kdTreeNClosestPoint2D(0.f, 0.f, 0, xs, ys, d2).empty());
  EXPECT_TRUE(xs.empty());
  c3.kdTreeNClosestPoint3DIdx(0.f, 0.f, 0.f, 0, idx, d2);
  EXPECT_TRUE(idx.empty());
  c3.kdTreeNClosestPoint3D(0.f, 0.f, 0.f, 0, xs, ys, zs, d2);
  EXPECT_TRUE(zs.empty());
  c3.kdTreeNClosestPoint3DWithIdx(0.f, 0.f, 0.f, 0, xs, ys, zs, idx, d2);
  EXPECT_TRUE(idx.empty());
}

// Saved and restored indices give the same k-NN results as brute force, for a
// random cloud, and the other dimension's tree is built on demand.
TEST(KDTreeCapable, saveLoadIndexRoundTripMatchesBruteForce)
{
#if !defined(MRPT_HAS_KDTREE_SAVE_LOAD_INDEX)
  GTEST_SKIP() << "Requires nanoflann >= v1.5.0";
#endif
  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);
  PointCloud3D a;
  for (int i = 0; i < 500; i++)
  {
    a.pts.emplace_back(rng.drawUniform(-5, 5), rng.drawUniform(-5, 5), rng.drawUniform(-1, 1));
  }
  std::stringstream ss3;
  std::stringstream ss2;
  ASSERT_TRUE(a.kdtree_save_index_3D(ss3));
  ASSERT_TRUE(a.kdtree_save_index_2D(ss2));

  const auto bruteKnn = [&](const mrpt::math::TPoint3D& q, size_t k, bool is3D)
  {
    std::vector<std::pair<double, size_t>> d;
    for (size_t i = 0; i < a.pts.size(); i++)
    {
      const auto& p = a.pts[i];
      const double d2 = mrpt::square(mrpt::d2f(p.x) - mrpt::d2f(q.x)) +
                        mrpt::square(mrpt::d2f(p.y) - mrpt::d2f(q.y)) +
                        (is3D ? mrpt::square(mrpt::d2f(p.z) - mrpt::d2f(q.z)) : 0.0);
      d.emplace_back(d2, i);
    }
    std::sort(d.begin(), d.end());
    std::vector<size_t> idx;
    for (size_t i = 0; i < k; i++)
    {
      idx.push_back(d[i].second);
    }
    return idx;
  };

  for (int pass = 0; pass < 2; pass++)
  {
    PointCloud3D b;
    b.pts = a.pts;
    if (pass == 0)
    {
      b.kdtree_load_index_3D(ss3);
    }
    else
    {
      b.kdtree_load_index_2D(ss2);
    }
    for (int iq = 0; iq < 50; iq++)
    {
      const mrpt::math::TPoint3D q(
          rng.drawUniform(-6, 6), rng.drawUniform(-6, 6), rng.drawUniform(-1.5, 1.5));
      std::vector<size_t> idx3;
      std::vector<float> d3;
      b.kdTreeNClosestPoint3DIdx(mrpt::d2f(q.x), mrpt::d2f(q.y), mrpt::d2f(q.z), 5, idx3, d3);
      EXPECT_EQ(idx3, bruteKnn(q, 5, true)) << "pass " << pass;
      std::vector<size_t> idx2;
      std::vector<float> d2;
      b.kdTreeNClosestPoint2DIdx(mrpt::d2f(q.x), mrpt::d2f(q.y), 5, idx2, d2);
      EXPECT_EQ(idx2, bruteKnn(q, 5, false)) << "pass " << pass;
    }
  }

  // A cloud with a different number of points must reject the index:
  PointCloud3D c;
  c.pts = a.pts;
  c.pts.pop_back();
  std::stringstream ss3b;
  ASSERT_TRUE(a.kdtree_save_index_3D(ss3b));
  EXPECT_THROW(c.kdtree_load_index_3D(ss3b), std::exception);
}
