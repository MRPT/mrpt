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

/** Correctness of the collision look-up table of the collision-grid-based
 *  PTGs, and of the robot-shape bookkeeping it relies on:
 *  - Soundness: for any obstacle point, the free distance read from the grid
 *    never exceeds the distance at which the continuously swept (possibly
 *    non-convex) footprint first touches it (brute-force ground truth).
 *  - Obstacles beyond refDistance but within reach of the footprint count.
 *  - The circumscribed radius follows shapes loaded from config files and
 *    streams, not only those set with setRobotShape().
 *  - getPathStepForDist() matches a linear scan.
 */

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/math/TPolygon2D.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/nav/tpspace/CPTG_DiffDrive_C.h>
#include <mrpt/random/RandomGenerators.h>
#include <mrpt/serialization/CArchive.h>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace mrpt::nav;

namespace
{
// L-shaped (non-convex) footprint:
mrpt::math::CPolygon lShape()
{
  mrpt::math::CPolygon p;
  p.add_vertex(-0.20, -0.20);
  p.add_vertex(0.35, -0.20);
  p.add_vertex(0.35, 0.05);
  p.add_vertex(0.05, 0.05);
  p.add_vertex(0.05, 0.20);
  p.add_vertex(-0.20, 0.20);
  return p;
}

std::shared_ptr<CPTG_DiffDrive_C> makeC(const mrpt::math::CPolygon& shape)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string s = "PTG";
  cfg.write(s, "num_paths", 21);
  cfg.write(s, "refDistance", 2.0);
  cfg.write(s, "resolution", 0.10);
  cfg.write(s, "v_max_mps", 1.0);
  cfg.write(s, "w_max_dps", 90.0);
  cfg.write(s, "K", 1.0);
  auto ptg = std::make_shared<CPTG_DiffDrive_C>();
  ptg->loadFromConfigFile(cfg, s);
  ptg->setRobotShape(shape);
  ptg->initialize(std::string(), false /*verbose*/);
  return ptg;
}

double pointPolygonDistance(const mrpt::math::TPoint2D& q, const mrpt::math::TPolygon2D& poly)
{
  if (poly.contains(q))
  {
    return 0;
  }
  double d = std::numeric_limits<double>::max();
  const size_t N = poly.size();
  for (size_t i = 0; i < N; i++)
  {
    const auto& a = poly[i];
    const auto& b = poly[(i + 1) % N];
    const double abx = b.x - a.x;
    const double aby = b.y - a.y;
    const double t =
        std::clamp(((q.x - a.x) * abx + (q.y - a.y) * aby) / (abx * abx + aby * aby), 0.0, 1.0);
    d = std::min(d, std::hypot(q.x - a.x - t * abx, q.y - a.y - t * aby));
  }
  return d;
}

mrpt::math::TPolygon2D placed(const mrpt::math::CPolygon& shape, const mrpt::math::TPose2D& p)
{
  mrpt::math::TPolygon2D out;
  for (size_t i = 0; i < shape.size(); i++)
  {
    const double x = shape.get_vertex_x(i);
    const double y = shape.get_vertex_y(i);
    out.emplace_back(
        p.x + std::cos(p.phi) * x - std::sin(p.phi) * y,
        p.y + std::sin(p.phi) * x + std::cos(p.phi) * y);
  }
  return out;
}
}  // namespace

TEST(PTGCollisionGrid, free_distance_is_sound_for_non_convex_footprint)
{
  const auto shape = lShape();
  const auto ptg = makeC(shape);

  auto& rng = mrpt::random::getRandomGenerator();
  rng.randomize(1234);

  const double range = ptg->getRefDistance() + 0.6;
  constexpr int kSubSteps = 8;
  size_t nContacts = 0;

  for (int i = 0; i < 150; i++)
  {
    const mrpt::math::TPoint2D o(rng.drawUniform(-range, range), rng.drawUniform(-range, range));

    // Obstacles already touching the robot are handled by the collision
    // behavior policy, not by the look-up table:
    if (pointPolygonDistance(o, placed(shape, {0, 0, 0})) <= 1e-3)
    {
      continue;
    }

    for (uint16_t k = 0; k < ptg->getPathCount(); k++)
    {
      double gridFree = 0;
      ptg->initTPObstacleSingle(k, gridFree);
      ptg->updateTPObstacleSingle(o.x, o.y, k, gridFree);

      // Brute-force first contact along the densely interpolated path:
      double trueFirstContact = std::numeric_limits<double>::max();
      const size_t nSteps = ptg->getPathStepCount(k);
      for (size_t n = 0; n + 1 < nSteps && trueFirstContact > 1e9; n++)
      {
        const auto p0 = ptg->getPathPose(k, static_cast<uint32_t>(n));
        const auto p1 = ptg->getPathPose(k, static_cast<uint32_t>(n + 1));
        for (int s = 0; s < kSubSteps; s++)
        {
          const double t = static_cast<double>(s) / kSubSteps;
          const mrpt::math::TPose2D p(
              p0.x + t * (p1.x - p0.x), p0.y + t * (p1.y - p0.y),
              p0.phi + t * mrpt::math::angDistance(p0.phi, p1.phi));
          if (placed(shape, p).contains(o))
          {
            trueFirstContact = ptg->getPathDist(k, static_cast<uint32_t>(n));
            break;
          }
        }
      }
      if (trueFirstContact > 1e9)
      {
        continue;
      }
      nContacts++;
      EXPECT_LE(gridFree, trueFirstContact + 1e-6) << "k=" << k << " obstacle=" << o.asString();
    }
  }
  EXPECT_GT(nContacts, 50U);  // the test is not vacuous
}

TEST(PTGCollisionGrid, obstacle_beyond_refDistance_blocks_path)
{
  const auto ptg = makeC(lShape());

  // Straight-ahead path: the footprint front ends 0.35 m past refDistance.
  const auto kStraight = static_cast<uint16_t>(ptg->getPathCount() / 2);
  double gridFree = 0;
  ptg->initTPObstacleSingle(kStraight, gridFree);
  ptg->updateTPObstacleSingle(ptg->getRefDistance() + 0.2, 0.0, kStraight, gridFree);
  EXPECT_LT(gridFree, ptg->getRefDistance());
}

TEST(PTGCollisionGrid, max_radius_follows_config_and_stream_shapes)
{
  mrpt::config::CConfigFileMemory cfg;
  const std::string s = "PTG";
  cfg.write(s, "num_paths", 11);
  cfg.write(s, "refDistance", 1.0);
  cfg.write(s, "resolution", 0.25);
  cfg.write(s, "v_max_mps", 1.0);
  cfg.write(s, "w_max_dps", 60.0);
  cfg.write(s, "K", 1.0);
  cfg.write(s, "shape_x0", -0.3);
  cfg.write(s, "shape_y0", 0.4);
  cfg.write(s, "shape_x1", 0.3);
  cfg.write(s, "shape_y1", 0.4);
  cfg.write(s, "shape_x2", 0.3);
  cfg.write(s, "shape_y2", -0.4);
  cfg.write(s, "shape_x3", -0.3);
  cfg.write(s, "shape_y3", -0.4);

  CPTG_DiffDrive_C ptg;
  ptg.loadFromConfigFile(cfg, s);
  EXPECT_NEAR(ptg.getMaxRobotRadius(), 0.5, 1e-9);
  // Points inside the shape but beyond a stale radius must be detected:
  EXPECT_TRUE(ptg.isPointInsideRobotShape(0.25, 0.35));

  ptg.initialize(std::string(), false /*verbose*/);
  mrpt::io::CMemoryStream buf;
  auto arch = mrpt::serialization::archiveFrom(buf);
  arch << ptg;
  buf.Seek(0);
  CPTG_DiffDrive_C ptg2;
  arch >> ptg2;
  EXPECT_NEAR(ptg2.getMaxRobotRadius(), 0.5, 1e-9);
  EXPECT_TRUE(ptg2.isPointInsideRobotShape(0.25, 0.35));
}

TEST(PTGCollisionGrid, getPathStepForDist_matches_linear_scan)
{
  const auto ptg = makeC(lShape());
  for (uint16_t k = 0; k < ptg->getPathCount(); k++)
  {
    const size_t nSteps = ptg->getPathStepCount(k);
    const double maxDist = ptg->getPathDist(k, static_cast<uint32_t>(nSteps - 1));
    for (double dist = 0.0; dist <= maxDist + 0.1; dist += 0.037)
    {
      std::optional<uint32_t> expected;
      for (size_t n = 0; n + 1 < nSteps; n++)
      {
        if (ptg->getPathDist(k, static_cast<uint32_t>(n + 1)) >= dist)
        {
          expected = static_cast<uint32_t>(n);
          break;
        }
      }
      EXPECT_EQ(ptg->getPathStepForDist(k, dist), expected) << "k=" << k << " dist=" << dist;
    }
  }
}
