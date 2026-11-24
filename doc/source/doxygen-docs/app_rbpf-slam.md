\page app_rbpf-slam rbpf-slam

# rbpf-slam

Offline SLAM with Rao-Blackwellized Particle Filters (RBPF): each particle
holds a robot path hypothesis and its own map, built from a rawlog dataset.
It is a front-end for mrpt::slam::CMetricMapBuilderRBPF (through
mrpt::apps::RBPF_SLAM_App_Rawlog), so it supports any map type in
mrpt::maps::CMultiMetricMap: occupancy grids, point clouds, beacon maps
(range-only SLAM), gas concentration grids, etc. Use it for grid-mapping with
2D laser scans when loops must be closed and odometry is poor, or for
range-only SLAM with beacons.

## Usage

    rbpf-slam <config_file.ini> [dataset.rawlog]

If the rawlog is not given on the command line, it is taken from
`rawlog_file` in the `[MappingApplication]` section. Both rawlog formats
(action-observation pairs and observations only) are accepted. Press `ESC` in
the console to stop processing early.

## Example

Run from the config directory, since the dataset paths are relative:

    cd modules/mrpt_data/config_files/rbpf-slam
    rbpf-slam gridmapping_RBPF_grid_ICPbased_malaga.ini   # grid mapping, 2D laser
    rbpf-slam gridmapping_optimal_sampling.ini            # grid mapping, optimal sampling
    rbpf-slam RO-SLAM_simulatedData_MC.ini                # range-only SLAM (beacons)

## Configuration file

All options are in the `[MappingApplication]` section:

- `rawlog_file`, `rawlog_offset` (number of initial entries to skip).
- `logOutput_dir` (default: `log_out`; its previous contents are deleted),
  `LOG_FREQUENCY` (save logs every N steps).
- `SAVE_MAP_IMAGES`, `SAVE_3D_SCENE`, `SAVE_POSE_LOG`, `GENERATE_LOG_INFO`
  (map entropy log, requires a grid map), `GENERATE_LOG_JOINT_H` (joint
  path and map entropy log).
- `SHOW_PROGRESS_IN_WINDOW`, `SHOW_PROGRESS_IN_WINDOW_DELAY_MS`,
  `PROGRESS_WINDOW_WIDTH`, `PROGRESS_WINDOW_HEIGHT`,
  `CAMERA_3DSCENE_FOLLOWS_ROBOT`: live 3D view.
- `RANDOM_SEED`: negative (default) to randomize from the current time.
- Continue mapping from an existing map: `SIMPLEMAP_CONTINUATION` (a
  `.simplemap` file), or `METRIC_MAP_CONTINUATION_GRIDMAP_FILE` (a `.gridmap`
  file) with `METRIC_MAP_CONTINUATION_START_POSE_{X,Y,PHI_DEG}`.
- Map update thresholds: `insertionLinDistance`, `insertionAngDistance_deg`,
  `localizeLinDistance`, `localizeAngDistance_deg`.
- Particle filter (mrpt::bayes::CParticleFilter::TParticleFilterOptions):
  `PF_algorithm` (0: standard proposal, 1: auxiliary PF, 2: optimal proposal
  (ICP-based for grid maps), 3: auxiliary PF with optimal sampling),
  `resamplingMethod`, `sampleSize`, `adaptiveSampleSize`, `BETA`, ...
- The metric maps (`occupancyGrid_count`, `pointsMap_count`, `beaconMap_count`,
  ...) and their options in sections `[MappingApplication_occupancyGrid_00_creationOpts]`,
  `[..._insertOpts]`, `[..._likelihoodOpts]`, as described in
  mrpt::maps::TSetOfMetricMapInitializers.

See mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions for the full list.

## Output

In `logOutput_dir`:

| File | Contents |
|---|---|
| `_finalmap_.simplemap` | Keyframes (poses and observations) of the most likely particle. |
| `finalMap*` | Final metric maps (e.g. grid map as `.png` plus limits). |
| `most_likely_path.txt` | Path of the most likely particle (`x y yaw` per row). |
| `log_estimated_path.txt`, `log_odometry_path.txt` | Mean estimated and odometry-only paths. |
| `log_times.txt`, `log_ParticlesStats.txt`, `log_MemoryUsage.txt` | Timing, number of particles and ESS, memory usage. |
| `mapping_NNNNN.png`, `maps/` | Map images and representations every `LOG_FREQUENCY` steps (`SAVE_MAP_IMAGES`). |
| `3D/buildingmap_NNNNN.3Dscene` | 3D scenes every `LOG_FREQUENCY` steps (`SAVE_3D_SCENE`). |

The `.simplemap` can be turned into other maps with
[observations2map](page_app_observations2map.html), inspected with
[robot-map-gui](page_app_robot-map-gui.html), or used for localization with
[pf-localization](page_app_pf-localization.html).

## See also

- [icp-slam](page_app_icp-slam.html): faster, but without loop closure.
- mrpt::slam::CMetricMapBuilderRBPF, mrpt::maps::CMultiMetricMapPDF
