\page app_kf-slam kf-slam


Offline Kalman Filter SLAM (EKF-SLAM) with range-bearing landmark
observations (mrpt::obs::CObservationBearingRange), read from a rawlog file.
It is a front-end for mrpt::slam::CRangeBearingKFSLAM (6D robot poses and 3D
landmarks) and mrpt::slam::CRangeBearingKFSLAM2D (2D poses and landmarks).
The filter can run as a standard EKF, an EKF that processes one scalar
observation at a time, or an iterated EKF (see mrpt::bayes::TKFMethod).

Typical uses: experimenting with EKF-SLAM and data association methods on
simulated datasets (see [simul-landmarks](page_app_simul-landmarks.html)), and
evaluating the results against ground truth.

# Usage

    kf-slam <config_file.ini> [dataset.rawlog]

If the rawlog is not given on the command line, it is taken from
`rawlog_file` in the `[MappingApplication]` section. Press `ESC` in the
console to stop processing early.

# Example

Sample config files and datasets are in `modules/mrpt_data` (installed under
`share/mrpt/`). The paths in the config files are relative, so run from the
config directory:

    cd modules/mrpt_data/config_files/kf-slam
    kf-slam EKF-SLAM_test.ini                 # 3D (6D poses) EKF-SLAM
    kf-slam EKF-SLAM_6D_test_datassoc.ini     # with unknown data association
    kf-slam EKF-SLAM_test_2d.ini ../../datasets/kf-slam_demo.rawlog   # 2D

# Configuration file

- `[MappingApplication]`: application options.
  - `rawlog_file`, `rawlog_offset` (number of initial entries to skip).
  - `kf_implementation`: `CRangeBearingKFSLAM` (default, 6D) or
    `CRangeBearingKFSLAM2D`.
  - `logOutput_dir` (default: `OUT_KF-SLAM`). Its previous contents are deleted.
  - `SAVE_LOG_FREQUENCY`: save the per-step output files every N steps.
  - `SAVE_3D_SCENES`, `SAVE_MAP_REPRESENTATIONS`, `SAVE_DA_LOG` (default: true).
  - `SHOW_3D_LIVE`, `CAMERA_3DSCENE_FOLLOWS_ROBOT`: live 3D view.
  - `ground_truth_file` (landmarks: rows with `x y z ... ID`, ID in column 7),
    `ground_truth_file_robot` (robot path: 6 columns `x y z yaw pitch roll`),
    `ground_truth_data_association`: optional, to evaluate the results.
- `[RangeBearingKFSLAM_KalmanFilter]`: generic Kalman filter options
  (mrpt::bayes::TKF_options), e.g. `method = kfEKFNaive | kfEKFAlaDavison | kfIKFFull`.
- `[RangeBearingKFSLAM]`: the SLAM options (motion and sensor noise, data
  association method and thresholds, whether to ignore odometry, etc.). See
  mrpt::slam::CRangeBearingKFSLAM::TOptions and
  mrpt::slam::CRangeBearingKFSLAM2D::TOptions.

# Output

In `logOutput_dir`:

| File | Contents |
|---|---|
| `robot_pose_NNNNN.txt` | Mean robot pose at each saved step. |
| `full_cov_NNNNN.txt` | Full covariance matrix of the filter state. |
| `slam_state_NNNNN.m` | MATLAB script drawing the map and path. |
| `kf_state_NNNNN.3Dscene` | 3D view of the filter state (open with [SceneViewer3D](page_app_SceneViewer3D.html)). |
| `data_association.log` | Data association decisions for each observed landmark. |
| `data_association_performance.log` | True/false positives/negatives vs. the data association ground truth. |

If a landmark ground truth file is given, the mean, minimum and maximum
landmark position errors are printed at the end.

# See also

- [simul-landmarks](page_app_simul-landmarks.html): generates datasets for this app.
- [2d-slam-demo](app_2d-slam-demo.html): interactive EKF-SLAM demo.
- mrpt::slam::CRangeBearingKFSLAM, mrpt::slam::CRangeBearingKFSLAM2D
