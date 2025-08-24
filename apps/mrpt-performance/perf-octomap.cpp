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

#include <mrpt/maps/COctoMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/stock_observations.h>
#include <mrpt/random.h>

#include "common.h"

// ------------------------------------------------------
//				Benchmark OctoMaps
// ------------------------------------------------------
double octomap_updateVoxel(int resolution_cm, int dist_points_cm)
{
  mrpt::maps::COctoMap map(resolution_cm * 0.01);

  auto& rnd = mrpt::random::getRandomGenerator();

  const size_t N = 100000;
  const double L = dist_points_cm * 0.01;

  mrpt::system::CTicTac tictac;
  for (unsigned n = 0; n < N; n++)
  {
    map.updateVoxel(rnd.drawUniform(-L, L), rnd.drawUniform(-L, L), rnd.drawUniform(-L, L), true);
  }
  return tictac.Tac() / N;
}

double octomap_insert2Dscan(int resolution_cm, int num_reps)
{
  auto& rnd = mrpt::random::getRandomGenerator();

  mrpt::obs::CObservation2DRangeScan scan1;
  mrpt::obs::stock_observations::example2DRangeScan(scan1);

  const double L = 2.0;  // [meters]

  mrpt::maps::COctoMap map(resolution_cm * 0.01);

  mrpt::system::CTicTac tictac;
  for (int n = 0; n < num_reps; n++)
  {
    mrpt::poses::CPose3D robot_pose(
        rnd.drawUniform(-L, L), rnd.drawUniform(-L, L), rnd.drawUniform(-L * 0.1, L * 0.1),
        rnd.drawUniform(-M_PI, M_PI), 0, 0);

    map.insertObservation(scan1, robot_pose);
  }
  return tictac.Tac() / num_reps;
}

// ------------------------------------------------------
// register_tests_octomaps
// ------------------------------------------------------
void register_tests_octomaps()
{
  lstTests.emplace_back(
      "octomap: updateVoxel() random, voxel=0.05m pts=5.0m", octomap_updateVoxel, 5, 500);
  lstTests.emplace_back(
      "octomap: updateVoxel() random, voxel=0.1m pts=5.0m", octomap_updateVoxel, 10, 500);
  lstTests.emplace_back(
      "octomap: updateVoxel() random, voxel=0.25m pts=5.0m", octomap_updateVoxel, 25, 500);

  lstTests.emplace_back("octomap: insert2Dscan(), voxel=0.05m", octomap_insert2Dscan, 5, 100);
  lstTests.emplace_back("octomap: insert2Dscan(), voxel=0.10m", octomap_insert2Dscan, 10, 100);
  lstTests.emplace_back("octomap: insert2Dscan(), voxel=0.25m", octomap_insert2Dscan, 25, 100);
}
