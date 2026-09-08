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

//! Minimal, headless walk-through of the TP-Space transformation of a PTG.

#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/nav/tpspace/CParameterizedTrajectoryGenerator.h>

#include <iostream>

using namespace mrpt::nav;

int main()
{
  try
  {
    // 1) Build a PTG by class name, from parameters in a (here, in-memory)
    //    config file. See each PTG class docs for its own parameters.
    mrpt::config::CConfigFileMemory cfg;
    const std::string sect = "PTG";
    cfg.write(sect, "num_paths", 61);     // how many discrete `alpha` values
    cfg.write(sect, "refDistance", 4.0);  // trajectories are cut at 4 m
    cfg.write(sect, "T_ramp_max", 0.8);
    cfg.write(sect, "v_max_mps", 1.0);
    cfg.write(sect, "w_max_dps", 60.0);
    cfg.write(sect, "robot_radius", 0.35);

    auto ptg = CParameterizedTrajectoryGenerator::CreatePTG("CPTG_Holo_Blend", cfg, sect, "");

    // initialize() is where collision look-up tables get built (or loaded from
    // a cache file, for the grid-based PTG families):
    ptg->initialize();

    std::cout << "PTG: " << ptg->getDescription() << "\n"
              << " paths       : " << ptg->getPathCount() << "\n"
              << " ref distance: " << ptg->getRefDistance() << " m\n\n";

    // 2) Workspace -> TP-Space: each obstacle point shortens the collision-free
    //    length of every path it blocks.
    const std::vector<mrpt::math::TPoint2D> obstacles = {
        {1.5,  0.0},
        {1.6,  0.3},
        {1.6, -0.3},
        {2.5,  1.6}
    };

    std::vector<double> tp_obstacles;
    ptg->initTPObstacles(tp_obstacles);  // all free: refDistance everywhere

    ClearanceDiagram clearance;
    ptg->initClearanceDiagram(clearance);

    for (const auto& o : obstacles)
    {
      ptg->updateTPObstacle(o.x, o.y, tp_obstacles);
      ptg->updateClearance(o.x, o.y, clearance);
    }

    // 3) In TP-Space the robot is a point free to move in any direction, so
    //    picking a motion is just picking the best `k`. Here: the longest
    //    collision-free path.
    uint16_t best_k = 0;
    for (uint16_t k = 1; k < ptg->getPathCount(); k++)
    {
      if (tp_obstacles[k] > tp_obstacles[best_k]) best_k = k;
    }

    std::cout << "Freest direction: k=" << best_k
              << " (alpha=" << mrpt::RAD2DEG(ptg->index2alpha(best_k)) << " deg)\n"
              << " collision-free : " << tp_obstacles[best_k] << " m\n"
              << " clearance      : "
              << clearance.getClearance(best_k, 1.0, ClearanceQuery::MeanUpToDistance) << "\n";

    // The pose the robot would reach at the end of that free stretch:
    if (const auto step = ptg->getPathStepForDist(best_k, tp_obstacles[best_k]); step)
    {
      std::cout << " end pose       : " << ptg->getPathPose(best_k, *step).asString() << "\n";
    }

    // 4) TP-Space -> Workspace: the velocity command that follows path `k`.
    const auto cmd = ptg->directionToMotionCommand(best_k);
    std::cout << " velocity cmd   : " << cmd->asString() << "\n\n";

    // 5) The inverse map answers "which path passes through this point?":
    const mrpt::math::TPoint2D query(1.0, 0.5);
    if (const auto tp = ptg->inverseMap_WS2TP(query.x, query.y); tp)
    {
      const auto [k, normalized_d] = *tp;
      std::cout << "Point " << query.asString() << " lies on path k=" << k
                << " at normalized distance " << normalized_d << "\n";
    }
    else
    {
      std::cout << "Point " << query.asString() << " is outside the PTG domain\n";
    }

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
