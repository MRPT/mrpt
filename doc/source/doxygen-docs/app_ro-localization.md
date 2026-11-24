\page app_ro-localization ro-localization

# ro-localization

Range-only localization with a particle filter, for robots measuring ranges
to fixed radio beacons (e.g. UWB) at known positions. Unlike plain Monte
Carlo localization ([pf-localization](page_app_pf-localization.html)), each
particle also estimates a bias for every beacon, which may change suddenly
over time (e.g. because of non-line-of-sight propagation). The particles are
implemented by a `CPosePDFParticlesExtended` class inside the app. For the
method, see https://www.mrpt.org/paperuwb-particle-filter-localization/

## Usage

    ro-localization <config_file.ini>

## Example

    cd modules/mrpt_data/config_files/ro-localization
    ro-localization UWB_localization_demo.ini

A 3D window shows the beacons, the particles and the estimated path while
the dataset is processed.

## Configuration file

- `[ro-localization]`:
  - `rawlog_file`: the dataset, with odometry and
    mrpt::obs::CObservationBeaconRanges observations.
  - `gridmap_file`: optional grid map (`.gridmap`), only for display.
  - `groundTruthFile`: optional ground truth path, to compute errors.
  - `logOutput_dir`: prefix of the output directories.
  - `3DSceneFrequency`: save a 3D scene every N steps.
  - `experimentRepetitions`, `random_seed` (0: randomize from the current time).
  - `Pc_range_ini`, `Pc_range_end`, `Pc_range_step`: optional sweep of the
    probability of a bias change, to run one experiment per value.
  - `SAVE_3D_TO_VIDEO`: grab the 3D window frames to image files.
- `[RO_MODEL]`: the bias model: `probabilityChangingBias` (probability of a
  bias change at each step), `changingBiasUnifRange` (range of the uniform
  distribution of new biases), `mixtureProposalRatio`.
- `[OdometryEstimation]`: initial robot pose (`odometry_X`, `odometry_Y`,
  `odometry_PHI` in degrees), in the beacons reference frame.
- `[PF_options]`: particle filter options; `sampleSize` is the number of
  particles (mrpt::bayes::CParticleFilter::TParticleFilterOptions).
- `[KLD_options]`: KLD adaptive sample size parameters.
- `[MetricMap]` and `[MetricMap_landmarksMap_00_*]`: the reference map with
  the beacon IDs and positions (`nBeacons`, `beacon_NNN_ID`,
  `beacon_NNN_{X,Y,Z}`) and the range likelihood options.

## Output

In `<logOutput_dir>_Pc_<value>/REP_NNN/`, for each repetition:
`3Dscene_NNN.3Dscene` (3D views every `3DSceneFrequency` steps),
`particle_matrix.txt` (estimated pose, its covariance, the ground truth pose
and the estimated beacon biases at each step), and `GT_ranges.txt` and
`GT_offsets.txt` (ranges computed from the ground truth, and measured minus
ground truth ranges). When sweeping `Pc_range_*`, the mean and standard
deviation of the position error for each value are appended to
`Pc_<ini>_<end>_RESULTs.txt`.

## See also

- [pf-localization](page_app_pf-localization.html)
- [rbpf-slam](page_app_rbpf-slam.html): range-only SLAM with unknown beacon positions.
- mrpt::bayes::CParticleFilter, mrpt::obs::CObservationBeaconRanges
