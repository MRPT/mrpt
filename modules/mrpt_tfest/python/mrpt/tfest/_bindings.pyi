"""
Python bindings for mrpt_tfest (Transformation Estimation)
"""
from __future__ import annotations
import mrpt.math
import typing
__all__: list[str] = ['TMatchingPair', 'TMatchingPairList', 'TMatchingPairList_d', 'TMatchingPair_d', 'TSE3RobustParams', 'TSE3RobustResult', 'se2_l2', 'se3_l2_robust']
class TMatchingPair:
    errorSquareAfterTransformation: float
    globalIdx: int
    global_pt: mrpt.math.TPoint3Df
    localIdx: int
    local_pt: mrpt.math.TPoint3Df
    def __init__(self) -> None:
        ...
class TMatchingPair_d:
    errorSquareAfterTransformation: float
    globalIdx: int
    global_pt: mrpt.math.TPoint3D
    localIdx: int
    local_pt: mrpt.math.TPoint3D
    def __init__(self) -> None:
        ...
class TMatchingPairList:
    def __init__(self) -> None:
        ...
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def push_back(self, arg0: TMatchingPair) -> None:
        ...
    def size(self) -> int:
        ...
class TMatchingPairList_d:
    def __init__(self) -> None:
        ...
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def push_back(self, arg0: TMatchingPair_d) -> None:
        ...
    def size(self) -> int:
        ...
class TSE3RobustParams:
    ransac_maxSetSizePct: float
    ransac_minSetSize: int
    ransac_nmaxSimulations: int
    ransac_threshold_lin: float
    user_individual_compat_callback: typing.Any  # unnamed C++ type
    def __init__(self) -> None:
        ...
class TSE3RobustResult:
    inliers_idx: list[int]
    scale: float
    def __init__(self) -> None:
        ...
    @property
    def transformation(self) -> typing.Any:  # unnamed C++ type
        """
        Estimated SE(3) transform as CPose3D (converted from internal CPose3DQuat).
        """
    @transformation.setter
    def transformation(self, arg1: typing.Any) -> None:  # unnamed C++ type
        ...
def se2_l2(arg0: TMatchingPairList) -> tuple:
    """
    Least-squares SE(2) estimation. Returns (ok, mrpt::math::TPose2D)
    """
def se3_l2_robust(arg0: TMatchingPairList, arg1: TSE3RobustParams) -> tuple:
    """
    Robust SE(3) estimation using RANSAC. Returns (ok, TSE3RobustResult)
    """
