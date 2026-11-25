\page app_simul-landmarks simul-landmarks


Generates a synthetic rawlog dataset of a robot moving among randomly placed
point landmarks, observed by a range-bearing sensor
(mrpt::obs::CObservationBearingRange) with configurable noise, field of view
and range. Noisy odometry is stored as mrpt::obs::CActionRobotMovement2D
actions (or mrpt::obs::CActionRobotMovement3D for 6D paths), and the ground
truth robot path is saved too. The output is the input expected by
[kf-slam](page_app_kf-slam.html), so it is useful to test EKF-SLAM and data
association algorithms with known ground truth.

# Usage

    simul-landmarks <config_file.ini>

# Example

    simul-landmarks modules/mrpt_data/config_files/simul-landmarks/simul_landmarks_demo_EKF-SLAM.ini
    kf-slam modules/mrpt_data/config_files/kf-slam/EKF-SLAM_test_2d.ini OUT/kf-slam_demo.rawlog

Other sample files in the same directory simulate 6D paths, a sensor not
centered on the robot, or sensors that don't detect landmark IDs (for testing
data association).

# Configuration file

`[Params]` section:

- `random_seed`: 0 to randomize from the current time, otherwise a fixed seed.
- `nSteps`: number of simulated time steps.
- Path shape: `circularPath` (circular) or a square path of side
  `squarePathLength` steps; `random6DPath` for a random 6D path instead.
- `odometryNoiseXY_std` (meters), `odometryNoisePhi_std_deg`,
  `odometryNoisePitch_std_deg`, `odometryNoiseRoll_std_deg` (degrees).
- Sensor: `minSensorDistance`, `maxSensorDistance`, `fieldOfView_deg`,
  `sensorDetectsIDs` (whether observations carry the landmark ID),
  sensor pose on the robot `sensorPose_{x,y,z,yaw_deg,pitch_deg,roll_deg}`,
  and noise `stdRange`, `stdYaw_deg`, `stdPitch_deg`.
- `outDir`, `outFile`: output directory and rawlog file name.
- `show_in_3d`: show the simulated landmarks and path in a 3D window.

Landmarks are generated from consecutive sections `[RANDOMSET_1]`,
`[RANDOMSET_2]`, ..., each with `nLandmarks` uniformly distributed within
`min_x`, `max_x`, `min_y`, `max_y`, `min_z`, `max_z`. Alternatively,
`loadLandmarksFromFile` loads them from a text file, one `X Y Z` row per
landmark.

# Output

In `outDir`:

- `<outFile>`: the simulated dataset (gz-compressed rawlog, in the
  action-observation format, see [rawlog format](rawlog_format.html)).
- `<outFile>_ground_truth_robot_path.txt`: ground truth robot poses, one row
  per step with columns `x y z yaw pitch roll`.
- `<outFile>_ground_truth.txt`: ground truth landmarks (`X Y Z TYPE
  TIMES_SEEN TIME_OF_LAST_OBSERVATION ID` per row), as read by the
  `ground_truth_file` option of [kf-slam](page_app_kf-slam.html).

# See also

- [kf-slam](page_app_kf-slam.html)
- mrpt::maps::CLandmarksMap
