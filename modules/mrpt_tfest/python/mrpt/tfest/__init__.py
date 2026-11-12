"""
mrpt-tfest Python API
"""
import mrpt.math  # noqa: F401  (TPoint2D/TPoint3D used in matching pairs)
import mrpt.poses  # noqa: F401  (CPose3D results)

from . import _bindings as _b

# Export Classes
TMatchingPair = _b.TMatchingPair
TMatchingPairList = _b.TMatchingPairList
TMatchingPair_d = _b.TMatchingPair_d
TMatchingPairList_d = _b.TMatchingPairList_d
TPotentialMatch = _b.TPotentialMatch
TSE2RobustParams = _b.TSE2RobustParams
TSE2RobustResult = _b.TSE2RobustResult
TSE3RobustParams = _b.TSE3RobustParams
TSE3RobustResult = _b.TSE3RobustResult

# Export Functions
se2_l2 = _b.se2_l2
se2_l2_with_covariance = _b.se2_l2_with_covariance
se2_l2_robust = _b.se2_l2_robust
se3_l2 = _b.se3_l2
se3_l2_robust = _b.se3_l2_robust


__all__ = [
    'TMatchingPair',
    'TMatchingPairList',
    'TMatchingPair_d',
    'TMatchingPairList_d',
    'TPotentialMatch',
    'TSE2RobustParams',
    'TSE2RobustResult',
    'TSE3RobustParams',
    'TSE3RobustResult',
    'se2_l2',
    'se2_l2_with_covariance',
    'se2_l2_robust',
    'se3_l2',
    'se3_l2_robust'
]
