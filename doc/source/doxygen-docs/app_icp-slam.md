\page app_icp-slam icp-slam


![icp-slam screenshot](https://mrpt.github.io/docs-assets/apps/app_icp-slam.webp)

`icp-slam` builds a metric map (point cloud and/or occupancy grid) from a
dataset (rawlog file) by aligning each new laser scan against the map built so
far with ICP (Iterative Closest Point). It is a thin wrapper of
mrpt::slam::CMetricMapBuilderICP, with ICP from mrpt::slam::CICP. It suits
datasets with good odometry and a 2D (or planar 3D) LiDAR, when a simple and
fast map is enough (no loop closure).

The companion app [icp-slam-live](page_app_icp-slam-live.html) runs the same
algorithm on live LiDAR data.

# Usage

    icp-slam <config_file.ini> [<dataset.rawlog>]

- `config_file.ini`: the configuration file (see below).
- `dataset.rawlog`: optional; if given, it overrides the `rawlog_file` entry of
  the configuration file.

# Example

    cd modules/mrpt_data/config_files/icp-slam
    icp-slam icp-slam_demo_classic.ini

Other sample configurations in the same directory use Levenberg-Marquardt ICP
(`icp-slam_demo_LM.ini`), grid matching (`icp-slam_demo_classic_gridmatch.ini`),
a voxel map (`icp-slam_demo_voxelmap.ini`) or an octomap
(`icp-slam_demo_with_octomap.ini`).

# Configuration file

- `[MappingApplication]`: the app and the map builder.
  - `rawlog_file`, `rawlog_offset` (entries to skip at the beginning).
  - `logOutput_dir`: output directory. **Its previous contents are deleted.**
  - `LOG_FREQUENCY`, `SAVE_3D_SCENE`, `SAVE_POSE_LOG`: save a 3D scene and/or
    the current pose PDF every `LOG_FREQUENCY` steps.
  - `SHOW_PROGRESS_3D_REAL_TIME`, `SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS`,
    `SHOW_LASER_SCANS_3D`: live 3D view.
  - `localizationLinDistance`, `localizationAngDistance`,
    `insertionLinDistance`, `insertionAngDistance`, `minICPgoodnessToAccept`,
    `matchAgainstTheGrid` and the rest of
    mrpt::slam::CMetricMapBuilderICP::TConfigParams.
  - The maps to build (`pointsMap_count`, `occupancyGrid_count`,...), with
    their options in sections like `[MappingApplication_pointsMap_00_insertOpts]`,
    as described in mrpt::maps::CMultiMetricMap.
  - `alwaysInsertByClass`: observation classes inserted into the map at every
    step.
- `[ICP]`: ICP parameters (mrpt::slam::CICP::TConfigParams), e.g.
  `ICP_algorithm`, `maxIterations`, `thresholdDist`, `thresholdAng_DEG`,
  `ALFA`, `corresponding_points_decimation`.

# Output

In the `logOutput_dir` directory:

| File | Contents |
|---|---|
| `_finalmap_.simplemap` | The final map as a simple map (poses and observations), e.g. for \ref app_observations2map or RawLogViewer. |
| `_finalmaps_.txt_*` | The final metric maps (e.g. point clouds as text, grid maps as images). |
| `log_estimated_path.txt` | Estimated robot path. |
| `log_odometry_path.txt` | Odometry-only robot path. |
| `log_times.txt` | Processing time per step. |
| `buildingmap_NNNNN.3Dscene` | 3D scenes of the map being built (if `SAVE_3D_SCENE`), viewable with [SceneViewer3D](page_app_SceneViewer3D.html). |
| `mapbuild_posepdf_NNN.txt` | Pose PDF per step (if `SAVE_POSE_LOG`). |

# See also

- [icp-slam-live](page_app_icp-slam-live.html), [rbpf-slam](page_app_rbpf-slam.html).
- Tutorial: [ICP alignment](tutorial-icp-alignment.html).
- mrpt::slam::CMetricMapBuilderICP, mrpt::slam::CICP.
