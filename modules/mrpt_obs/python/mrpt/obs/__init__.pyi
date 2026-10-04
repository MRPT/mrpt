"""
mrpt.obs — Python bindings for MRPT sensor observations and actions.
"""
from __future__ import annotations
import mrpt as mrpt
from mrpt.obs._bindings import CAction as CAction
from mrpt.obs._bindings import CActionCollection as CActionCollection
from mrpt.obs._bindings import CActionRobotMovement2D as CActionRobotMovement2D
from mrpt.obs._bindings import CActionRobotMovement3D as CActionRobotMovement3D
from mrpt.obs._bindings import CObservation as CObservation
from mrpt.obs._bindings import CObservation2DRangeScan as CObservation2DRangeScan
from mrpt.obs._bindings import CObservation3DRangeScan as CObservation3DRangeScan
from mrpt.obs._bindings import CObservationGPS as CObservationGPS
from mrpt.obs._bindings import CObservationIMU as CObservationIMU
from mrpt.obs._bindings import CObservationImage as CObservationImage
from mrpt.obs._bindings import CObservationOdometry as CObservationOdometry
from mrpt.obs._bindings import CObservationRobotPose as CObservationRobotPose
from mrpt.obs._bindings import CRawlog as CRawlog
from mrpt.obs._bindings import CSensoryFrame as CSensoryFrame
from mrpt.obs._bindings import CSimpleMap as CSimpleMap
from mrpt.obs._bindings import GnssFixType as GnssFixType
from mrpt.obs._bindings import T3DPointsProjectionParams as T3DPointsProjectionParams
from mrpt.obs._bindings import TIMUDataIndex as TIMUDataIndex
from mrpt.obs._bindings import TMapGenericParams as TMapGenericParams
from mrpt.obs._bindings import TMetricMapInitializer as TMetricMapInitializer
from mrpt.obs._bindings import TSetOfMetricMapInitializers as TSetOfMetricMapInitializers
from mrpt.obs._bindings import gnss as gnss
from . import _bindings
__all__: list = ['CObservation', 'CObservation2DRangeScan', 'CObservation3DRangeScan', 'T3DPointsProjectionParams', 'CObservationGPS', 'GnssFixType', 'gnss', 'CObservationImage', 'CObservationIMU', 'TIMUDataIndex', 'CObservationOdometry', 'CObservationRobotPose', 'CAction', 'CActionRobotMovement2D', 'CActionRobotMovement3D', 'CActionCollection', 'CSensoryFrame', 'CRawlog', 'CSimpleMap', 'TMapGenericParams', 'TMetricMapInitializer', 'TSetOfMetricMapInitializers']
