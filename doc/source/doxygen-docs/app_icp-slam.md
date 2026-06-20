\page  app_icp-slam Application: icp-slam

[TOC]

# Overview

`icp-slam` is a command-line application that builds a 2D metric map (occupancy grid and/or point cloud) from a recorded dataset (rawlog file) using **ICP-based SLAM** (Iterative Closest Point Simultaneous Localization and Mapping).

It is based on `mrpt::slam::CMetricMapBuilderICP` and supports:
- Occupancy grid map building from 2D laser scans.
- Optional 3D visualization of the robot trajectory and the growing map.
- Saving the resulting map and robot path to files.

A companion application, `icp-slam-live`, provides the same functionality but reads sensor data in real-time from hardware drivers instead of a rawlog file.

# Usage

```
icp-slam  [--help] <config_file.ini> [dataset.rawlog]
```

Arguments:

| Argument | Description |
|---|---|
| `config_file.ini` | Path to an INI-style configuration file (see below). |
| `dataset.rawlog` | (Optional) Path to the rawlog file. Overrides the value in the config file. |

# Configuration file

The configuration file controls all aspects of the SLAM algorithm and output.
Key sections and parameters:

```ini
[ICP_SLAM_App]
# Rawlog file to process (can be overridden from the command line)
rawlog_file = path/to/dataset.rawlog

# Number of rawlog entries to process (0 = all)
rawlog_offset = 0

# Output directory for maps and paths
logOutput_dir = ./icp-slam-results

# Log frequency (save map every N steps, 0 = never)
LOG_FREQUENCY = 5

# Whether to show a 3D window during processing
SHOW_PROGRESS_3D_REAL_TIME = true

[MetricMaps]
# Define the map(s) to build.  See mrpt::maps::CMultiMetricMap documentation.
occupancyGrid_count = 1

[MetricMaps_occupancyGrid_00_creationOpts]
resolution = 0.05

[ICP]
# ICP parameters. See mrpt::slam::CICP::TConfigParams.
thresholdDist   = 0.75
thresholdAng    = 0.15
```

For a complete list of parameters, run the application with `--help` or refer to
`mrpt::slam::CMetricMapBuilderICP::TConfigParams` and
`mrpt::slam::CICP::TConfigParams`.

# Output files

Results are written to the directory specified by `logOutput_dir`:

| File | Contents |
|---|---|
| `log_*.rawlog` | Sub-sequence rawlog (optional). |
| `final_map.simplemap` | Final serialized metric map. |
| `robot_path.txt` | Robot trajectory as a sequence of 2D poses. |
| `occupancy_grid.png` | Rendered occupancy grid image. |

# See also

- \ref mrpt::slam::CMetricMapBuilderICP
- \ref mrpt::slam::CICP
- Application `icp-slam-live` for live sensor input.
