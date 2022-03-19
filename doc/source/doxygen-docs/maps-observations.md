\page maps_observations Maps and observations compatibility matrices

There exists many kinds of metric maps and observations in MRPT, but some operations
involving a map and an observation (for example "inserting" an observation in a map to update it)
only make sense for a small subset of map-observation combinations.

See:

- List of all observations: see derived classes from mrpt::obs::CObservation
- List of all metric maps: see derived classes from mrpt::maps::CMetricMap

The following tables summarize the valid combinations, as implemented so far:

# Valid implementations of insertObservation()

List of observations and the metric maps in which mrpt::maps::CMetricMaps::insertObservation() is 
implemented for them:

- mrpt::obs::CObservation2DRangeScan
  - mrpt::maps::CColouredOctoMap
  - mrpt::maps::CHeightGridMap2D
  - mrpt::maps::CHeightGridMap2D_MRF
  - mrpt::maps::COccupancyGridMap2D
  - mrpt::maps::COctoMap
  - mrpt::maps::CPointsMap (any derived map)
- mrpt::obs::CObservation3DRangeScan
  - mrpt::maps::CColouredOctoMap
  - mrpt::maps::COctoMap
  - mrpt::maps::CPointsMap (any derived map)
- mrpt::obs::CObservation6DFeatures
  - (none)
- mrpt::obs::CObservationBeaconRanges
  - mrpt::maps::CBeaconMap (see [RO-SLAM](http://www.mrpt.org/tutorials/slam-algorithms/rangeonly_slam/))                                                                                         |
- mrpt::obs::CObservationBearingRange
  - (none)
- mrpt::obs::CObservationGasSensors
  - mrpt::maps::CGasConcentrationGridMap2D
- mrpt::obs::CObservationGPS
  - (none)
- mrpt::obs::CObservationImage
  - mrpt::maps::CLandmarksMap (Extract SIFT feats)
- mrpt::obs::CObservationIMU
  - (none)
- mrpt::obs::CObservationOdometry
  - (none)
- mrpt::obs::CObservationPointCloud
  - mrpt::maps::CPointsMap (any derived map) 
- mrpt::obs::CObservationRange
  - mrpt::maps::CPointsMap (any derived map)
  - mrpt::maps::COccupancyGridMap2D 
- mrpt::obs::CObservationReflectivity
  - mrpt::maps::CReflectivityGridMap2D  
- mrpt::obs::CObservationRFID
  - (none)
- mrpt::obs::CObservationRGBD360
  - (none)
- mrpt::obs::CObservationRobotPose
  - (none)
- mrpt::obs::CObservationStereoImages
  - mrpt::maps::CLandmarksMap (Extract SIFT feats)  
- mrpt::obs::CObservationStereoImagesFeatures
  - mrpt::maps::CLandmarksMap (Append/fuse landmarks) 
- mrpt::obs::CObservationVelodyneScan
  - mrpt::maps::CPointsMap (any derived map), mrpt::maps::CHeightGridMap2D, mrpt::maps::CHeightGridMap2D_MRF                                                                                      |
- mrpt::obs::CObservationWindSensor
  - (none)
- mrpt::obs::CObservationWirelessPower
  - mrpt::maps::CWirelessPowerGridMap2D 

# Valid implementations of computeObservationLikelihood()

Next follows the list of observations and the metric maps in which mrpt::maps::CMetricMaps::computeObservationLikelihood() is 
implemented for them. That means that, for example, particle filter algorithms can use 
those observations to localize a robot in the corresponding map, fusing the information from many sources if several 
observation-map pairs are valid for a given robot.

- mrpt::obs::CObservation2DRangeScan
  - mrpt::maps::CColouredOctoMap
  - mrpt::maps::CPointsMap (any derived map)
  - mrpt::maps::COccupancyGridMap2D
  - mrpt::maps::COctoMap
  - mrpt::maps::CLandmarksMap
- mrpt::obs::CObservation3DRangeScan
  - mrpt::maps::CColouredOctoMap and mrpt::maps::COctoMap (must have pointcloud)
- mrpt::obs::CObservation6DFeatures
  - (none)
- mrpt::obs::CObservationBeaconRanges
  - mrpt::maps::CBeaconMap (Used for SLAM)
  - mrpt::maps::CLandmarksMap (Used for localization-only)
- mrpt::obs::CObservationBearingRange
  - (none, but check out the EKF-SLAM algorithms)
- mrpt::obs::CObservationGasSensors
  - (none...check?)
- mrpt::obs::CObservationGPS
  - mrpt::maps::CLandmarksMap (NMEA GGA datum)
- mrpt::obs::CObservationImage
  - (none)
- mrpt::obs::CObservationIMU
  - (none)
- mrpt::obs::CObservationOdometry
  - (none, but it is explicitly used by most SLAM algorithms)
- mrpt::obs::CObservationPointCloud
  - mrpt::maps::CPointsMap (any derived map)
  - mrpt::maps::CColouredOctoMap & mrpt::maps::COctoMap
- mrpt::obs::CObservationRange
  - (none)
- mrpt::obs::CObservationReflectivity
  - mrpt::maps::CReflectivityGridMap2D
- mrpt::obs::CObservationRFID
  - (none)
- mrpt::obs::CObservationRGBD360
  - (none)
- mrpt::obs::CObservationRobotPose
  - mrpt::maps::CLandmarksMap
- mrpt::obs::CObservationStereoImages
  - mrpt::maps::CLandmarksMap (Convert to SIFT features)
- mrpt::obs::CObservationStereoImagesFeatures
  - (none)
- mrpt::obs::CObservationVelodyneScan
  - mrpt::maps::CPointsMap (any derived map)
  - mrpt::maps::CColouredOctoMap & mrpt::maps::COctoMap
- mrpt::obs::CObservationWindSensor
  - (none)
- mrpt::obs::CObservationWirelessPower
  - (none)
