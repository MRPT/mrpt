.. _tutorial-slam-algorithms:

===========================================================================
SLAM algorithms in MRPT
===========================================================================

Not all SLAM algorithms fit any kind of observation (sensor data) and produce any map type.
The following summarizes the SLAM algorithms implemented in MRPT and their associated map and observation types, 
grouped by input sensors.

- 2D laser scanner `mrpt::obs::CObservation2DRangeScan <class_mrpt_obs_CObservation2DRangeScan.html>`_:
   - To generate 2D occupancy grids (`mrpt::maps::COccupancyGridMap2D <class_mrpt_maps_COccupancyGridMap2D.html>`_) 
     or point clouds (`mrpt::maps::CPointsMap <class_mrpt_maps_CPointsMap.html>`_).

     - `rbpf-slam <page_app_rbpf-slam.html>`_
     - `icp-slam <page_app_icp-slam.html>`_

- Sonar sensors `mrpt::obs::CObservationRange <class_mrpt_obs_CObservationRange.html>`_:
   - To generate 2D occupancy grids (`mrpt::maps::COccupancyGridMap2D <class_mrpt_maps_COccupancyGridMap2D.html>`_):

     - `rbpf-slam <page_app_rbpf-slam.html>`_

- Range-bearing landmarks (`mrpt::obs::CObservationBearingRange <class_mrpt_obs_CObservationBearingRange.html>`_):

  - EKF-based SLAM: `kf-slam <page_app_kf-slam.html>`_


- Monocular image features (visual keypoint matches from a visual SLAM front-end):

  - Back-end only: `vision_bundle_adj_example <page_vision_bundle_adj_example.html>`_


- Range-only sensors (`mrpt::obs::CObservationBeaconRanges <class_mrpt_obs_CObservationBeaconRanges.html>`_):

  - `Range-only SLAM page <range_only_localization_mapping.html>`_


- Relative poses (Pose-graph or Graph-SLAM):

  - Graph-SLAM maps (Write me!) https://www.mrpt.org/Graph-SLAM_maps

