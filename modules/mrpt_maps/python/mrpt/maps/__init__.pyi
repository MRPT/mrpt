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
from __future__ import annotations
import mrpt as mrpt
from mrpt.maps._bindings import CBeacon as CBeacon
from mrpt.maps._bindings import CBeaconMap as CBeaconMap
from mrpt.maps._bindings import CGenericPointsMap as CGenericPointsMap
from mrpt.maps._bindings import CHeightGridMap2D as CHeightGridMap2D
from mrpt.maps._bindings import CMetricMap as CMetricMap
from mrpt.maps._bindings import CMultiMetricMap as CMultiMetricMap
from mrpt.maps._bindings import CObservationPointCloud as CObservationPointCloud
from mrpt.maps._bindings import COccupancyGridMap2D as COccupancyGridMap2D
from mrpt.maps._bindings import COccupancyGridMap3D as COccupancyGridMap3D
from mrpt.maps._bindings import COctoMap as COctoMap
from mrpt.maps._bindings import CPointsMap as CPointsMap
from mrpt.maps._bindings import CSimplePointsMap as CSimplePointsMap
from mrpt.maps._bindings import CVoxelMap as CVoxelMap
from mrpt.maps._bindings import CVoxelMapRGB as CVoxelMapRGB
from mrpt.maps._bindings import PointCloudRecoloringParameters as PointCloudRecoloringParameters
from mrpt.maps._bindings import VisualizationParameters as VisualizationParameters
from mrpt.maps._bindings import obs_to_viz as obs_to_viz
from mrpt.obs._bindings import CSimpleMap as CSimpleMap
from mrpt.obs._bindings import TMapGenericParams as TMapGenericParams
from mrpt.obs._bindings import TMetricMapInitializer as TMetricMapInitializer
from mrpt.obs._bindings import TSetOfMetricMapInitializers as TSetOfMetricMapInitializers
from . import _bindings
__all__: list = ['CMetricMap', 'CPointsMap', 'CSimplePointsMap', 'CGenericPointsMap', 'COccupancyGridMap2D', 'COccupancyGridMap3D', 'CVoxelMap', 'CVoxelMapRGB', 'COctoMap', 'CHeightGridMap2D', 'CBeacon', 'CBeaconMap', 'CMultiMetricMap', 'CObservationPointCloud', 'PointCloudRecoloringParameters', 'VisualizationParameters', 'obs_to_viz', 'CSimpleMap', 'TMapGenericParams', 'TMetricMapInitializer', 'TSetOfMetricMapInitializers']
