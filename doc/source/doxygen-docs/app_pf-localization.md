\page app_pf-localization pf-localization

# pf-localization

Offline Monte Carlo localization (particle filter) of a robot in a known map,
processing a rawlog dataset. It is a front-end for
mrpt::slam::CMonteCarloLocalization2D (2D poses, the default) and
mrpt::slam::CMonteCarloLocalization3D (6D poses), through
mrpt::apps::MonteCarloLocalization_Rawlog. Any map type of
mrpt::maps::CMultiMetricMap can be used for the observation likelihood
(occupancy grids with 2D laser scans, beacon maps with range-only sensors,
point maps, ...). It supports a fixed number of particles or KLD adaptive
sample size, global localization from a uniform initial distribution, and
repeated experiments with statistics against a ground truth.

## Usage

    pf-localization <config_file.ini> [dataset.rawlog]

If the rawlog is not given on the command line, it is taken from
`rawlog_file` in the `[LocalizationExperiment]` section.

## Example

    cd modules/mrpt_data/config_files/pf-localization
    pf-localization localization_demo.ini

This performs global localization (40000 initial particles, KLD adaptive
sample size) of a robot with a 2D laser scanner in the grid map built from
`datasets/localization_demo.simplemap.gz`.

## Configuration file

- `[LocalizationExperiment]`:
  - `map_file`: the map, either a `.simplemap` (the maps defined in
    `[MetricMap]` are built from it) or a `.gridmap` file (`.gz` allowed).
  - `rawlog_file`, `rawlog_offset`, `use_3D_poses` (default: false).
  - `logOutput_dir` (mandatory): prefix of the output directories.
  - `particles_count` (mandatory): initial number of particles. A list
    (e.g. `100 1000 10000`) runs the experiment once per value.
  - `experimentRepetitions`: number of repetitions of each experiment.
  - `init_PDF_mode` (mandatory): 0 = uniform over the free cells of the grid
    map, 1 = uniform over the whole range. The range is given by
    `init_PDF_{min,max}_{x,y,z}` and `init_PDF_{min,max}_{yaw,pitch,roll}_deg`.
  - `ground_truth_path_file`: optional ground truth robot path, to compute errors.
  - `SHOW_PROGRESS_3D_REAL_TIME`, `SHOW_PROGRESS_3D_REAL_TIME_DELAY_MS`,
    `3DSceneFrequency`, `3DSceneFollowRobot`: live 3D view and saved scenes.
  - `SAVE_STATS_ONLY`: only save statistics, not particles nor 3D scenes.
  - `initial_odo_{x,y,phi}`: initial value of the accumulated odometry pose.
- `[PF_options]`: particle filter algorithm, resampling method, KLD adaptive
  sample size, etc. (mrpt::bayes::CParticleFilter::TParticleFilterOptions;
  `sampleSize` is ignored, `particles_count` is used instead).
- `[KLD_options]`: KLD adaptive sample size parameters
  (mrpt::slam::TKLDParams).
- `[DummyOdometryParams]`: odometry noise model for rawlogs in the
  observations-only format (`minStdXY`, `minStdPHI`, `additional_std_XYZ`,
  `additional_std_angle`).
- `[MetricMap]` and `[MetricMap_*]` sections: the metric maps and their
  likelihood options (see mrpt::maps::TSetOfMetricMapInitializers).

## Output

For each number of particles `N` and repetition `R`, a directory
`<logOutput_dir>_RRR_NNNNNNN/` with:

- `particles/particles_NNNNN.txt`: the particles at each step, and
  `3D/progress_NNNNN.3Dscene`: 3D views (unless `SAVE_STATS_ONLY`).
- `map*`: images/representations of the map.
- `PF_stats.txt`, `cov_est.txt`, `odo_est.txt`, `exec_times.txt`, and,
  with ground truth, `GT_error.txt` and `ODO_error.txt`.

A summary of all the experiments (convergence ratio, time, errors) is appended
to `<logOutput_dir>_SUMMARY.txt`.

## See also

- [rbpf-slam](page_app_rbpf-slam.html), [icp-slam](page_app_icp-slam.html):
  to build the map.
- [ro-localization](page_app_ro-localization.html): range-only localization
  with beacon bias estimation.
- mrpt::slam::CMonteCarloLocalization2D, mrpt::slam::CMonteCarloLocalization3D,
  mrpt::bayes::CParticleFilter
