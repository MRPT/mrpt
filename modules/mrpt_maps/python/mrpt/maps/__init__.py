"""
mrpt.maps — Metric map representations for MRPT.

Provides:
  - CMetricMap          : Abstract base class for all metric maps
  - CPointsMap          : Abstract base class for point cloud maps
  - CSimplePointsMap    : Concrete XYZ point cloud map
  - COccupancyGridMap2D : Probabilistic 2D occupancy grid map
"""

import mrpt.rtti          # noqa: F401
import mrpt.serialization  # noqa: F401
import mrpt.poses          # noqa: F401
import mrpt.obs            # noqa: F401

from mrpt.maps._bindings import (
    CMetricMap,
    CPointsMap,
    CSimplePointsMap,
    COccupancyGridMap2D,
)

__all__ = [
    "CMetricMap",
    "CPointsMap",
    "CSimplePointsMap",
    "COccupancyGridMap2D",
]
