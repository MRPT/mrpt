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

#include <mrpt/core/exceptions.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <CLI/CLI.hpp>
#include <fstream>
#include <iostream>

#include "SlamDemoApp.h"
#include "SlamDemoSimulator.h"

namespace
{
/** Runs a non-interactive experiment from a config file, and appends its
 * statistics to "slam_out.txt". */
void doBatchExperiment(SlamDemoSimulator& sim, const std::string& cfgFile, bool run)
{
  sim.loadConfig(cfgFile);
  sim.reset();

  if (!run)
  {
    return;  // only load the config: no statistics to report
  }
  const size_t nSteps = sim.defaultBatchSteps();
  mrpt::system::CTicTac const tim;
  for (size_t i = 0; i < nSteps; i++)
  {
    sim.step();
  }
  const auto st = sim.computeStats(tim.Tac());

  std::cout << "FP: " << st.meanFP << " FN: " << st.meanFN << " JCBB iters: " << st.meanJCBBIters
            << " Run iters: " << st.steps << "\n";
  std::cout << "Mean err_xy: " << st.meanErrXY << " mean tim: " << st.meanStepTime << "\n";

  std::ofstream f("slam_out.txt", std::ios::out | std::ios::app);
  f << st.meanStepTime << " " << st.meanErrXY << " " << st.meanErrPhi << " " << st.meanMahalanobis2
    << " " << st.meanJCBBIters << " " << st.meanFP << " " << st.meanFN << "\n";
}
}  // namespace

int main(int argc, char** argv)
{
  try
  {
    CLI::App cli("2d-slam-demo: interactive simulator of range-bearing EKF-SLAM");
    cli.set_version_flag("--version", mrpt::system::MRPT_getVersion());

    std::string cfgFile;
    cli.add_option("-c,--config", cfgFile, "Config file of a batch experiment to run");
    bool nogui = false;
    cli.add_flag("-n,--nogui", nogui, "Don't open the GUI: exit after the experiment.");
    bool norun = false;
    cli.add_flag("-r,--norun", norun, "Just load the config file, don't run it.");
    CLI11_PARSE(cli, argc, argv);

    SlamDemoSimulator sim;

    if (!cfgFile.empty())
    {
      if (!mrpt::system::fileExists(cfgFile))
      {
        std::cerr << "The indicated config file does not exist: " << cfgFile << "\n";
        return 1;
      }
      doBatchExperiment(sim, cfgFile, !norun);
      if (nogui)
      {
        return 0;
      }
    }
    else
    {
      sim.reset();
    }

    SlamDemoApp app(sim);
    return app.run();
  }
  catch (const std::exception& e)
  {
    std::cerr << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
