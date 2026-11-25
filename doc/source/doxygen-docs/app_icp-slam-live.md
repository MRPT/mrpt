\page app_icp-slam-live icp-slam-live


Real-time 2D ICP-based SLAM from a live 2D LIDAR. It runs the same algorithm
as [icp-slam](page_app_icp-slam.html) (mrpt::slam::CMetricMapBuilderICP,
through mrpt::apps::ICP_SLAM_App_Live), but instead of reading a rawlog file
it launches a sensor driver (any mrpt::hwdrivers::CGenericSensor that produces
mrpt::obs::CObservation2DRangeScan observations, e.g. RPLIDAR, Hokuyo, SICK)
in a background thread and maps with the most recent scan at each step. No
odometry is used.

# Usage

    icp-slam-live <config_file.ini>

The program stops when the sensor fails or no scan arrives for 2 seconds, or
when `ESC` is pressed in the console. The final map is then saved.

# Example

    icp-slam-live modules/mrpt_data/config_files/icp-slam-live/icp-slam-live_demo_RPLIDAR_gridmap.ini

Edit the serial port (`COM_port_LIN` / `COM_port_WIN`) and sensor pose in the
`[LIDAR_SENSOR]` section first.

# Configuration file

- `[LIDAR_SENSOR]`: the sensor driver. `driver` is the class name of the
  driver (e.g. `CRoboPeakLidar`, `CHokuyoURG`, `CSickLaserUSB`) and
  `process_rate` (Hz, must be above the sensor rate) is mandatory; the rest
  are the driver-specific parameters, as in the
  [rawlog-grabber](page_app_rawlog-grabber.html) config files (see
  `modules/mrpt_data/config_files/rawlog-grabber/`).
- `[MappingApplication]`: the same options as [icp-slam](page_app_icp-slam.html):
  - `logOutput_dir` (default: `log_out`; its previous contents are deleted),
    `LOG_FREQUENCY`, `SAVE_3D_SCENE`, `SAVE_POSE_LOG`.
  - `SAVE_RAWLOG`: also save the grabbed sensor data to a rawlog
    (`icpslamlive_dataset_<date>.rawlog`, in the current directory).
  - `SHOW_PROGRESS_3D_REAL_TIME`, `SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS`,
    `SHOW_LASER_SCANS_3D`: live 3D view of the map and robot.
  - Map update thresholds (`insertionLinDistance`, `insertionAngDistance`,
    `localizationLinDistance`, `localizationAngDistance`), `minICPgoodnessToAccept`,
    `matchAgainstTheGrid`, and the metric map definitions
    (`occupancyGrid_count`, `pointsMap_count`, and their
    `[MappingApplication_*]` sections). See
    mrpt::slam::CMetricMapBuilderICP::TConfigParams.
- `[ICP]`: the ICP parameters (mrpt::slam::CICP::TConfigParams).

# Output

In `logOutput_dir`:

| File | Contents |
|---|---|
| `_finalmap_.simplemap` | All the keyframes (poses and scans). |
| `_finalmaps_.txt*` | Final metric maps (e.g. grid map image, points as text). |
| `log_estimated_path.txt` | Estimated robot path (`step x y yaw` per row). |
| `log_times.txt`, `log_MemoryUsage.txt` | Execution time and memory usage per step. |
| `buildingmap_NNNNN.3Dscene` | 3D scenes every `LOG_FREQUENCY` steps (`SAVE_3D_SCENE`). |

# See also

- [icp-slam](page_app_icp-slam.html): the same, from a rawlog file.
- [rawlog-grabber](page_app_rawlog-grabber.html): to record datasets.
- mrpt::slam::CMetricMapBuilderICP, mrpt::slam::CICP
