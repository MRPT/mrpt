"""
mrpt.maps — Metric map representations for MRPT.

Provides:
  - CMetricMap          : Abstract base class for all metric maps
  - CPointsMap          : Abstract base class for point cloud maps
  - CSimplePointsMap    : Concrete XYZ point cloud map
  - CGenericPointsMap   : XYZ point cloud + arbitrary string-keyed data channels
  - COccupancyGridMap2D : Probabilistic 2D occupancy grid map
  - COccupancyGridMap3D : Dense probabilistic 3D occupancy grid
  - CVoxelMap, CVoxelMapRGB : Sparse voxel occupancy maps
  - COctoMap            : OctoMap-based 3D occupancy map
  - CHeightGridMap2D    : 2.5D elevation map
  - CBeaconMap, CBeacon : Map of range-only beacons
  - CMultiMetricMap     : Container of heterogeneous maps
  - CObservationPointCloud : Observation holding a point cloud map
  - VisualizationParameters, obs_to_viz() : 3D rendering of observations
  - CSimpleMap, TSetOfMetricMapInitializers, ... : re-exported from mrpt.obs
"""

import mrpt.rtti          # noqa: F401
import mrpt.serialization  # noqa: F401
import mrpt.config         # noqa: F401
import mrpt.poses          # noqa: F401
import mrpt.viz            # noqa: F401  (CSetOfObjects returned by getVisualization())
import mrpt.obs            # noqa: F401

from mrpt.maps._bindings import (
    CMetricMap,
    CPointsMap,
    CSimplePointsMap,
    CGenericPointsMap,
    COccupancyGridMap2D,
    COccupancyGridMap3D,
    CVoxelMap,
    CVoxelMapRGB,
    COctoMap,
    CHeightGridMap2D,
    CBeacon,
    CBeaconMap,
    CMultiMetricMap,
    CObservationPointCloud,
    PointCloudRecoloringParameters,
    VisualizationParameters,
    obs_to_viz,
)

# Defined in the mrpt_obs library (namespace mrpt::maps in C++):
from mrpt.obs import (
    CSimpleMap,
    TMapGenericParams,
    TMetricMapInitializer,
    TSetOfMetricMapInitializers,
)

__all__ = [
    "CMetricMap",
    "CPointsMap",
    "CSimplePointsMap",
    "CGenericPointsMap",
    "COccupancyGridMap2D",
    "COccupancyGridMap3D",
    "CVoxelMap",
    "CVoxelMapRGB",
    "COctoMap",
    "CHeightGridMap2D",
    "CBeacon",
    "CBeaconMap",
    "CMultiMetricMap",
    "CObservationPointCloud",
    "PointCloudRecoloringParameters",
    "VisualizationParameters",
    "obs_to_viz",
    "CSimpleMap",
    "TMapGenericParams",
    "TMetricMapInitializer",
    "TSetOfMetricMapInitializers",
]
