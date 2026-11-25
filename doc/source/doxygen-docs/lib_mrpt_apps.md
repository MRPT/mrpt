\defgroup mrpt_apps_grp [mrpt-apps]

Applications library

[TOC]

# Library mrpt-apps

This C++ library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-libapps-cli-dev libmrpt-libapps-gui-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

This library contains library wrappers for ready-to-use applications, for easy
integration into final programs, test suites, etc.

## Overview

Each class here is the full implementation of one MRPT program (for example,
\ref app_icp-slam is just a `main()` creating an
mrpt::apps::ICP_SLAM_App_Rawlog). Use them to run those algorithms from your
own programs or tests, change their parameters programmatically, and read
their results back without parsing output files.

The classes, all in the namespace mrpt::apps, are split into two packages:

- `mrpt_libapps_cli` (headers `<mrpt/apps_cli/...>`, CMake target
  `mrpt::mrpt_libapps_cli`): dataset tools without GUI dependencies.
- `mrpt_libapps_gui` (headers `<mrpt/apps_gui/...>`, CMake target
  `mrpt::mrpt_libapps_gui`): SLAM and localization apps, which can show
  their progress in 3D windows.

## Key classes

- mrpt::apps::ICP_SLAM_App_Rawlog, mrpt::apps::ICP_SLAM_App_Live: ICP-based
  SLAM from a dataset or from live sensors (\ref app_icp-slam,
  \ref app_icp-slam-live).
- mrpt::apps::RBPF_SLAM_App_Rawlog: Rao-Blackwellized particle filter SLAM
  (\ref app_rbpf-slam).
- mrpt::apps::KFSLAMApp: EKF-SLAM with range-bearing landmarks
  (\ref app_kf-slam).
- mrpt::apps::MonteCarloLocalization_Rawlog: particle filter localization
  (\ref app_pf-localization).
- mrpt::apps::RawlogEditApp, mrpt::apps::RawlogGrabberApp: the
  \ref app_rawlog-edit and \ref app_rawlog-grabber tools.

The typical usage is: `initialize(argc, argv)` with the same arguments as the
command-line program, optionally modify the public parameters (e.g. the INI
contents in `params`), then call `run()`.

## Example: running ICP-SLAM from a program

```cpp
#include <mrpt/apps_gui/ICP_SLAM_App.h>

int main(int argc, char** argv)
{
  // Takes the same command-line arguments as the icp-slam program:
  mrpt::apps::ICP_SLAM_App_Rawlog app;
  app.initialize(argc, argv);
  // `app.params` holds the loaded INI parameters, and can be modified here.
  app.run();

  for (const auto& [t, pose] : app.out_estimated_path)
  {
    std::cout << mrpt::Clock::toDouble(t) << " " << pose << "\n";
  }
}
```

# Library contents
