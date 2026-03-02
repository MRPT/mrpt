"""
mrpt.obs — Python bindings for MRPT sensor observations and actions.
"""

from . import _bindings as _b

# Base classes
CObservation = _b.CObservation
CAction = _b.CAction

# Range sensor observations
CObservation2DRangeScan = _b.CObservation2DRangeScan

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
CActionCollection = _b.CActionCollection

# Containers
CSensoryFrame = _b.CSensoryFrame

__all__ = [
    "CObservation",
    "CObservation2DRangeScan",
    "CObservationImage",
    "CObservationIMU",
    "TIMUDataIndex",
    "CObservationOdometry",
    "CObservationRobotPose",
    "CAction",
    "CActionRobotMovement2D",
    "CActionCollection",
    "CSensoryFrame",
]
