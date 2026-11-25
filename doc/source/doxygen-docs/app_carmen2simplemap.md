\page app_carmen2simplemap carmen2simplemap


Converts a CARMEN log text file (`*.log`) with **corrected** robot poses (for
example, the output of a SLAM method) into an MRPT simple map
(mrpt::maps::CSimpleMap, a `*.simplemap` file): a sequence of keyframes, each
one a robot pose plus the observations taken from it. The simple map can then
be turned into any kind of metric map with \ref app_observations2map, or used
as a prior map for localization.

# Usage

    carmen2simplemap -i <input.log> -o <output.simplemap> [-z <level>] [-w] [-q]

- `-i, --input`: input CARMEN log file (required).
- `-o, --output`: output simple map file (required).
- `-z, --compress-level`: zstd compression level of the output file, 1-22
  (default: 5).
- `-w, --overwrite`: overwrite the output file if it already exists (otherwise,
  the program refuses to run).
- `-q, --quiet`: terse output (no progress bar).
- `--version`, `-h, --help`.

# Example

    carmen2simplemap -i intel-corrected.log -o intel.simplemap

Then, build an occupancy grid map from it, with the map definition in one of
the sample config files in `mrpt_data`:

    observations2map config_files/icp-slam/icp-slam_demo_classic_gridmatch.ini intel.simplemap intel_map

# Input / output formats

The supported CARMEN lines are those of \ref app_carmen2rawlog (`ROBOTLASER*`,
`FLASER`, `RLASER` and `PARAM`). For each laser line, one keyframe is added to
the simple map:

- Pose: the robot pose in that line, taken as an exact pose (an
  mrpt::poses::CPose3DPDFGaussian with zero covariance).
- Observations: an mrpt::obs::CSensoryFrame with the
  mrpt::obs::CObservation2DRangeScan of that line.

Lines that do not provide a robot pose are skipped.

# See also

- \ref app_carmen2rawlog: converts CARMEN logs into rawlog datasets.
- \ref app_observations2map: builds metric maps from simple maps.
