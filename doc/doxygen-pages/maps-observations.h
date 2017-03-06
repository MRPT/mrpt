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

The following table summarizes the valid combinations, as implemented so far:


### Valid implementations of `computeObservationLikelihood()`

See: mrpt::maps::CMetricMaps::computeObservationLikelihood()

| Observations (below) \ Maps (right)     | mrpt::maps::CBeaconMap | mrpt::maps::CColouredOctoMap | mrpt::maps::CColouredPointsMap | mrpt::maps::CGasConcentrationGridMap2D | mrpt::maps::CHeightGridMap2D | mrpt::maps::CHeightGridMap2D_MRF | mrpt::maps::COccupancyGridMap2D | mrpt::maps::COctoMap | mrpt::maps::CReflectivityGridMap2D | mrpt::maps::CSimplePointsMap | mrpt::maps::CWeightedPointsMap | mrpt::maps::CWirelessPowerGridMap2D |
|-----------------------------------------|------------------------|------------------------------|--------------------------------|----------------------------------------|------------------------------|----------------------------------|---------------------------------|----------------------|------------------------------------|------------------------------|--------------------------------|-------------------------------------|
|-----------------------------------------|------------------------|------------------------------|--------------------------------|----------------------------------------|------------------------------|----------------------------------|---------------------------------|----------------------|------------------------------------|------------------------------|--------------------------------|-------------------------------------|


### Valid implementations of `insertObservation()`

See: mrpt::maps::CMetricMaps::insertObservation()


*/
