\page app_carmen2rawlog carmen2rawlog


Converts a CARMEN log text file (`*.log`) into an MRPT dataset
([rawlog](rawlog_format.html) file), so that classic CARMEN datasets can be
used with MRPT SLAM and localization apps, or inspected with
[RawLogViewer](app_RawLogViewer.html) and \ref app_rawlog-edit.

# Usage

    carmen2rawlog -i <input.log> -o <output.rawlog> [-p <period>] [-z <level>] [-w] [-q]

- `-i, --input`: input CARMEN log file (required).
- `-o, --output`: output rawlog file (required).
- `-p, --period`: time between consecutive log lines, in seconds (default: 0.1).
  See the note on timestamps below.
- `-z, --compress-level`: compression level of the output file (default: 5).
  The output is written with zstd compression.
- `-w, --overwrite`: overwrite the output file if it already exists (otherwise,
  the program refuses to run).
- `-q, --quiet`: terse output (no progress bar).
- `--version`, `-h, --help`.

# Example

    carmen2rawlog -i intel.log -o intel.rawlog

# Input / output formats

Each supported CARMEN line (see mrpt::obs::carmen_log_parse_line()) becomes
one or more observations, saved in the "observations-only" rawlog format:

- `ROBOTLASER1`, `ROBOTLASER2`, `FLASER`, `RLASER`: one
  mrpt::obs::CObservation2DRangeScan plus one mrpt::obs::CObservationOdometry
  with the robot pose of that line. The laser pose on the robot is computed
  from the laser and robot poses in the line.
- `PARAM` lines: only `robot_front_laser_max`, `laser_front_laser_resolution`,
  `robot_rear_laser_max` and `laser_rear_laser_resolution` are used, for the
  `FLASER` and `RLASER` lines that follow.
- `ODOM` and `RAWLASER` lines are ignored.

Timestamps: the timestamps in the CARMEN file are not used. The first line is
stamped with the current time, and each following line `--period` seconds
later.

Ground truth: the robot poses of all laser lines are also written to a text
file named after the output file with the extension `.gt.txt` (e.g.
`intel.gt.txt`), one `timestamp x y phi` row per pose. These are only
meaningful as ground truth if the CARMEN log contains corrected poses (e.g.
the output of a SLAM method).

# See also

- \ref app_carmen2simplemap: converts CARMEN logs with corrected poses into
  simple maps.
- \ref app_rawlog-edit, [RawLogViewer](app_RawLogViewer.html).
