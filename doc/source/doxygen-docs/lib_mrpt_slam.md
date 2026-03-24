\defgroup mrpt_slam_grp [mrpt_slam]

SLAM and PF-localization algorithms

[TOC]

# Library mrpt_slam

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-slam-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

`mrpt_slam` provides SLAM and probabilistic localization algorithms:

## Localization

- mrpt::slam::CMonteCarloLocalization2D: Particle filter (Monte Carlo) localization for a robot in a planar scenario.
- mrpt::slam::CMonteCarloLocalization3D: 3D particle-filter localization.

## Metric-map-based SLAM

- mrpt::slam::CMetricMapBuilder: Virtual base for ICP and RBPF-based SLAM.
- mrpt::slam::CMetricMapBuilderICP: Incremental ICP-based mapping.
- mrpt::slam::CMetricMapBuilderRBPF: Rao-Blackwellized Particle Filter SLAM.

## Landmark-based (EKF) SLAM

- mrpt::slam::CRangeBearingKFSLAM: Range-bearing EKF SLAM in 3D.
- mrpt::slam::CRangeBearingKFSLAM2D: Range-bearing EKF SLAM in 2D.

## Multi-metric maps

- mrpt::maps::CMultiMetricMap: Contains an arbitrary combination of metric maps
  (occupancy grids, point clouds, landmark maps, etc.) updated simultaneously.

## Data association

The `data_association.h` header provides both NN and JCBB algorithms as
generic templates — suitable for any measurement/landmark combination.

See also: For Graph-SLAM, see the namespace mrpt::graphslam in `mrpt_graphslam`.

# Library contents
