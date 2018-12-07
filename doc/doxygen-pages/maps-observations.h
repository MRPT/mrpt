/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005 2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page maps_observations Maps and observations compatibility matrix
 *

There exists many kinds of metric maps and observations in MRPT, but some operations
involving a map and an observation (for example "inserting" an observation in a map to update it)
only make sense for a small subset of map-observation combinations.

See:
- List of all observations: see derived classes from mrpt::obs::CObservation
- List of all metric maps: see derived classes from mrpt::maps::CMetricMap

## Map-observation compatibility matrix

The following tables summarize the valid combinations, as implemented so far:


### Valid implementations of `insertObservation()`

See: mrpt::maps::CMetricMaps::insertObservation()

| Observations                                | Maps                                                                                                                                                                                          |
|---------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| mrpt::obs::CObservation2DRangeScan          | mrpt::maps::CColouredOctoMap, mrpt::maps::CHeightGridMap2D, mrpt::maps::CHeightGridMap2D_MRF, mrpt::maps::COccupancyGridMap2D, mrpt::maps::COctoMap, mrpt::maps::CPointsMap (any derived map) |
| mrpt::obs::CObservation3DRangeScan          | mrpt::maps::CColouredOctoMap, mrpt::maps::COctoMap, mrpt::maps::CPointsMap (any derived map)                                                                                                  |
| mrpt::obs::CObservation6DFeatures           |                                                                                                                                                                                               |
| mrpt::obs::CObservationBeaconRanges         | mrpt::maps::CBeaconMap (see [RO-SLAM](http://www.mrpt.org/tutorials/slam-algorithms/rangeonly_slam/))                                                                                         |
| mrpt::obs::CObservationBearingRange         |                                                                                                                                                                                               |
| mrpt::obs::CObservationGasSensors           | mrpt::maps::CGasConcentrationGridMap2D                                                                                                                                                        |
| mrpt::obs::CObservationGPS                  |                                                                                                                                                                                               |
| mrpt::obs::CObservationImage                | mrpt::maps::CLandmarksMap (Extract SIFT feats)                                                                                                                                                |
| mrpt::obs::CObservationIMU                  |                                                                                                                                                                                               |
| mrpt::obs::CObservationOdometry             |                                                                                                                                                                                               |
| mrpt::obs::CObservationPointCloud           | mrpt::maps::CPointsMap (any derived map)                                                                                                                                                      |
| mrpt::obs::CObservationRange                | mrpt::maps::CPointsMap (any derived map), mrpt::maps::COccupancyGridMap2D                                                                                                                     |
| mrpt::obs::CObservationReflectivity         | mrpt::maps::CReflectivityGridMap2D                                                                                                                                                            |
| mrpt::obs::CObservationRFID                 |                                                                                                                                                                                               |
| mrpt::obs::CObservationRGBD360              |                                                                                                                                                                                               |
| mrpt::obs::CObservationRobotPose            |                                                                                                                                                                                               |
| mrpt::obs::CObservationStereoImages         | mrpt::maps::CLandmarksMap (Extract SIFT feats)                                                                                                                                                |
| mrpt::obs::CObservationStereoImagesFeatures | mrpt::maps::CLandmarksMap (Append/fuse landmarks)                                                                                                                                             |
| mrpt::obs::CObservationVelodyneScan         | mrpt::maps::CPointsMap (any derived map), mrpt::maps::CHeightGridMap2D, mrpt::maps::CHeightGridMap2D_MRF                                                                                      |
| mrpt::obs::CObservationWindSensor           |                                                                                                                                                                                               |
| mrpt::obs::CObservationWirelessPower        | mrpt::maps::CWirelessPowerGridMap2D                                                                                                                                                           |


### Valid implementations of `computeObservationLikelihood()`

See: mrpt::maps::CMetricMaps::computeObservationLikelihood()

| Observations                                | Maps                                                                                                                                                         |
|---------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------|
| mrpt::obs::CObservation2DRangeScan          | mrpt::maps::CColouredOctoMap, mrpt::maps::CPointsMap (any derived map), mrpt::maps::COccupancyGridMap2D, mrpt::maps::COctoMap, mrpt::maps::CLandmarksMap     |
| mrpt::obs::CObservation3DRangeScan          | mrpt::maps::CColouredOctoMap & mrpt::maps::COctoMap  (must have pointcloud)                                                                                  |
| mrpt::obs::CObservation6DFeatures           |                                                                                                                                                              |
| mrpt::obs::CObservationBeaconRanges         | mrpt::maps::CBeaconMap	 (Used for SLAM), mrpt::maps::CLandmarksMap (Used for localization-only)                                                             |
| mrpt::obs::CObservationBearingRange         |                                                                                                                                                              |
| mrpt::obs::CObservationGasSensors           |                                                                                                                                                              |
| mrpt::obs::CObservationGPS                  | mrpt::maps::CLandmarksMap (NMEA GGA datum)                                                                                                                   |
| mrpt::obs::CObservationImage                |                                                                                                                                                              |
| mrpt::obs::CObservationIMU                  |                                                                                                                                                              |
| mrpt::obs::CObservationOdometry             |                                                                                                                                                              |
| mrpt::obs::CObservationPointCloud           | mrpt::maps::CPointsMap (any derived map), mrpt::maps::CColouredOctoMap & mrpt::maps::COctoMap                                                                |
| mrpt::obs::CObservationRange                |                                                                                                                                                              |
| mrpt::obs::CObservationReflectivity         | mrpt::maps::CReflectivityGridMap2D                                                                                                                           |
| mrpt::obs::CObservationRFID                 |                                                                                                                                                              |
| mrpt::obs::CObservationRGBD360              |                                                                                                                                                              |
| mrpt::obs::CObservationRobotPose            | mrpt::maps::CLandmarksMap                                                                                                                                    |
| mrpt::obs::CObservationStereoImages         | mrpt::maps::CLandmarksMap (Convert to SIFT features)                                                                                                         |
| mrpt::obs::CObservationStereoImagesFeatures |                                                                                                                                                              |
| mrpt::obs::CObservationVelodyneScan         | mrpt::maps::CPointsMap (any derived map), mrpt::maps::CColouredOctoMap & mrpt::maps::COctoMap                                                                |
| mrpt::obs::CObservationWindSensor           |                                                                                                                                                              |
| mrpt::obs::CObservationWirelessPower        |                                                                                                                                                              |


*/
