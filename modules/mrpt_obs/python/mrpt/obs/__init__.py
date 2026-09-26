"""
mrpt.obs — Python bindings for MRPT sensor observations and actions.
"""

import mrpt.rtti          # noqa: F401
import mrpt.serialization  # noqa: F401
import mrpt.config         # noqa: F401 (CLoadableOptions, base of the map definitions)
import mrpt.math           # noqa: F401 (TTwist3D, CMatrixDouble33, ...)
import mrpt.img            # noqa: F401 (CImage, TCamera in camera observations)
import mrpt.poses          # noqa: F401 (CPose3D used in sensor poses)

from . import _bindings as _b

# Base classes
CObservation = _b.CObservation
CAction = _b.CAction

# Range sensor observations
CObservation2DRangeScan = _b.CObservation2DRangeScan
CObservation3DRangeScan = _b.CObservation3DRangeScan
T3DPointsProjectionParams = _b.T3DPointsProjectionParams

# GNSS
CObservationGPS = _b.CObservationGPS
GnssFixType = _b.GnssFixType
gnss = _b.gnss

# Camera observations
CObservationImage = _b.CObservationImage

# IMU
CObservationIMU = _b.CObservationIMU
TIMUDataIndex = _b.TIMUDataIndex

# Odometry / Pose
CObservationOdometry = _b.CObservationOdometry
CObservationRobotPose = _b.CObservationRobotPose

# Actions
CActionRobotMovement2D = _b.CActionRobotMovement2D
CActionRobotMovement3D = _b.CActionRobotMovement3D
CActionCollection = _b.CActionCollection

# Containers and datasets
CSensoryFrame = _b.CSensoryFrame
CRawlog = _b.CRawlog

# Metric map definitions and view-based maps (namespace mrpt::maps in C++, but
# part of the mrpt_obs library). Also re-exported by mrpt.maps.
CSimpleMap = _b.CSimpleMap
TMapGenericParams = _b.TMapGenericParams
TMetricMapInitializer = _b.TMetricMapInitializer
TSetOfMetricMapInitializers = _b.TSetOfMetricMapInitializers

__all__ = [
    "CObservation",
    "CObservation2DRangeScan",
    "CObservation3DRangeScan",
    "T3DPointsProjectionParams",
    "CObservationGPS",
    "GnssFixType",
    "gnss",
    "CObservationImage",
    "CObservationIMU",
    "TIMUDataIndex",
    "CObservationOdometry",
    "CObservationRobotPose",
    "CAction",
    "CActionRobotMovement2D",
    "CActionRobotMovement3D",
    "CActionCollection",
    "CSensoryFrame",
    "CRawlog",
    "CSimpleMap",
    "TMapGenericParams",
    "TMetricMapInitializer",
    "TSetOfMetricMapInitializers",
]
