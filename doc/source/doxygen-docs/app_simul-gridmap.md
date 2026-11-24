\page app_simul-gridmap simul-gridmap

# simul-gridmap

Generates a synthetic dataset of a robot that follows a given sequence of
poses in a world defined by an occupancy grid map, taking a 2D laser scan at
each pose (ray tracing in the grid, with range and bearing noise). Use it to
test 2D SLAM, localization or scan matching methods with known ground truth.

## Usage

    simul-gridmap -g <map.gridmap.gz> -p <poses.txt> -o <out.rawlog> [-i <in.rawlog>]
                  [-r <ranges>] [-s <span>] [-R <std_r>] [-B <std_b>] [-n]

- `-g, --grid`: the grid map (`*.gridmap` or `*.gridmap.gz`, a serialized
  mrpt::maps::COccupancyGridMap2D, e.g. created with \ref app_image2gridmap).
  Required.
- `-p, --poses`: text file with the robot poses, one `time x y phi` row per
  pose (meters and radians; the `time` column is not used). Required.
- `-o, --out-rawlog`: the output rawlog file. Required.
- `-i, --in-rawlog`: optional input rawlog, to re-simulate its laser scans
  (see below).
- `-r, --ranges`: number of ranges per scan (default: 361).
- `-s, --span`: angular span of the scans, in degrees (default: 180). Note: in
  the current version, new scans always span 180 degrees.
- `-R, --std_r`: range noise, 1 sigma, in meters (default: 0.01).
- `-B, --std_b`: bearing noise, 1 sigma, in degrees (default: 0.05).
- `-n, --nologo`: do not print the program banner.
- `--version`, `-h, --help`.

## Example

Create a grid map from a sample image in `mrpt_data`, then simulate scans
along the poses in `poses.txt`:

    image2gridmap -i datasets/graphslam-engine-demos/basic_map.png -r 0.05 -o basic_map.gridmap.gz
    simul-gridmap -g basic_map.gridmap.gz -p poses.txt -o simul.rawlog

with a `poses.txt` like:

    0.0  0.0  0.0  0.0
    0.1  0.1  0.0  0.0
    0.2  0.2  0.0  0.05

## Output format

The output rawlog is in the "actions-sensory frames" format (see the
[rawlog format](rawlog_format.html)):

- Without `--in-rawlog`: an initial mrpt::obs::CActionCollection, followed by
  one mrpt::obs::CSensoryFrame (with the simulated
  mrpt::obs::CObservation2DRangeScan) and one mrpt::obs::CActionCollection
  per pose, starting at the second pose of the file. Timestamps are the
  wall-clock time of the simulation.
- With `--in-rawlog`: the input rawlog is copied to the output, with the
  ranges of the mrpt::obs::CObservation2DRangeScan in each sensory frame
  replaced by simulated ones at the corresponding pose, so the odometry and
  timestamps of the input dataset are kept. The input rawlog must be in the
  "actions-sensory frames" format, start with an action, and have exactly one
  action and one sensory frame per row of the poses file.

## See also

- \ref app_image2gridmap, \ref app_ros-map-yaml2mrpt: create grid maps.
- \ref app_simul-beacons, \ref app_simul-landmarks: other dataset simulators.
- mrpt::maps::COccupancyGridMap2D::laserScanSimulator().
