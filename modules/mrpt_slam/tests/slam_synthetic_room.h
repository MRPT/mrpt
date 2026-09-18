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
#pragma once

/** Helpers shared by the SLAM unit tests: a synthetic closed room used as
 *  ground truth, plus 2D laser scans and odometry actions simulated from it,
 *  so that no dataset files are needed.
 */

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/datetime.h>

namespace mrpt::test
{
/** A closed 10x10 m rectangular room, used as ground truth to simulate scans. */
inline const mrpt::maps::COccupancyGridMap2D& groundTruthRoom()
{
  static mrpt::maps::COccupancyGridMap2D grid = []()
  {
    mrpt::maps::COccupancyGridMap2D g(-5.0f, 5.0f, -5.0f, 5.0f, 0.05f);
    g.fill(0.9f);  // all free
    for (unsigned int cx = 0; cx < g.getSizeX(); cx++)
    {
      g.setCell(cx, 0, 0.0f);
      g.setCell(cx, g.getSizeY() - 1, 0.0f);
    }
    for (unsigned int cy = 0; cy < g.getSizeY(); cy++)
    {
      g.setCell(0, cy, 0.0f);
      g.setCell(g.getSizeX() - 1, cy, 0.0f);
    }
    return g;
  }();
  return grid;
}

/** Successive calls return timestamps 100 ms apart. Do not use Clock::now()
 *  for each simulated step: its resolution is coarse enough on some platforms
 *  that consecutive calls return the *same* value, and consumers that need a
 *  strictly increasing clock (e.g. CRobot2DPoseEstimator) then drop the
 *  updates.
 */
inline mrpt::system::TTimeStamp nextTimestamp()
{
  static mrpt::system::TTimeStamp t = mrpt::Clock::now();
  t = mrpt::system::timestampAdd(t, 0.1);
  return t;
}

inline mrpt::obs::CObservation2DRangeScan::Ptr simulateScan(
    const mrpt::poses::CPose2D& robotPose, const mrpt::system::TTimeStamp t)
{
  auto scan = mrpt::obs::CObservation2DRangeScan::Create();
  scan->aperture = 2 * M_PIf;
  scan->maxRange = 20.0f;
  scan->sensorLabel = "LASER";
  scan->timestamp = t;
  groundTruthRoom().laserScanSimulator(
      *scan, robotPose, 0.5f /*threshold*/, 181 /*N*/, 0.0f /*noiseStd*/);
  return scan;
}

inline mrpt::obs::CSensoryFrame::Ptr simulateSF(
    const mrpt::poses::CPose2D& robotPose, const mrpt::system::TTimeStamp t)
{
  auto sf = mrpt::obs::CSensoryFrame::Create();
  sf->insert(simulateScan(robotPose, t));
  return sf;
}

inline mrpt::obs::CActionCollection::Ptr makeOdometryAction(
    const mrpt::poses::CPose2D& increment, const mrpt::system::TTimeStamp t)
{
  mrpt::obs::CActionRobotMovement2D act;
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions opts;
  opts.modelSelection = mrpt::obs::CActionRobotMovement2D::mmGaussian;
  opts.gaussianModel.minStdXY = 0.02;
  opts.gaussianModel.minStdPHI = mrpt::DEG2RAD(0.5);
  act.computeFromOdometry(increment, opts);
  act.timestamp = t;

  auto acts = mrpt::obs::CActionCollection::Create();
  acts->insert(act);
  return acts;
}
}  // namespace mrpt::test
