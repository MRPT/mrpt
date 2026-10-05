"""
Python bindings for mrpt_tfest (Transformation Estimation)
"""
from __future__ import annotations
import mrpt.math
import mrpt.poses
import typing
__all__: list[str] = ['TMatchingPair', 'TMatchingPairList', 'TMatchingPairList_d', 'TMatchingPair_d', 'TPotentialMatch', 'TSE3RobustParams', 'TSE3RobustResult', 'se2_l2', 'se3_l2_robust']
class TMatchingPair:
    """
    A pair of corresponding points (float coordinates) in two point sets.
    """
    errorSquareAfterTransformation: float
    globalIdx: int
    global_pt: mrpt.math.TPoint3Df
    localIdx: int
    local_pt: mrpt.math.TPoint3Df
    def __init__(self) -> None:
        """
        Default constructor.
        """
class TMatchingPair_d:
    """
    A pair of corresponding points (double coordinates) in two point sets.
    """
    errorSquareAfterTransformation: float
    globalIdx: int
    global_pt: mrpt.math.TPoint3D
    localIdx: int
    local_pt: mrpt.math.TPoint3D
    def __init__(self) -> None:
        """
        Default constructor.
        """
class TMatchingPairList:
    """
    A list of TMatchingPair.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def push_back(self, arg0: TMatchingPair) -> None:
        """
        Appends a pair.
        """
    def size(self) -> int:
        """
        Returns the number of pairs.
        """
class TMatchingPairList_d:
    """
    A list of TMatchingPair_d.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __iter__(self) -> typing.Iterator:
        ...
    def __len__(self) -> int:
        ...
    def push_back(self, arg0: TMatchingPair_d) -> None:
        """
        Appends a pair.
        """
    def size(self) -> int:
        """
        Returns the number of pairs.
        """
class TPotentialMatch:
    """
    A potential pairing of two points, passed to user_individual_compat_callback.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def idx_other(self) -> int:
        """
        Index of the point in the other set
        """
    @idx_other.setter
    def idx_other(self, arg0: int) -> None:
        ...
    @property
    def idx_this(self) -> int:
        """
        Index of the point in this set
        """
    @idx_this.setter
    def idx_this(self, arg0: int) -> None:
        ...
class TSE3RobustParams:
    """
    Parameters of se3_l2_robust().
    """
    ransac_maxSetSizePct: float
    ransac_minSetSize: int
    ransac_nmaxSimulations: int
    ransac_threshold_lin: float
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def user_individual_compat_callback(self) -> typing.Callable[[TPotentialMatch], bool]:
        """
        Optional function(TPotentialMatch) -> bool: return True to accept a candidate pairing, False to reject it (None: accept all)
        """
    @user_individual_compat_callback.setter
    def user_individual_compat_callback(self, arg0: typing.Callable[[TPotentialMatch], bool]) -> None:
        ...
class TSE3RobustResult:
    """
    Result of se3_l2_robust().
    """
    inliers_idx: list[int]
    scale: float
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def transformation(self) -> mrpt.poses.CPose3D:
        """
        Estimated SE(3) transform as CPose3D (converted from internal CPose3DQuat).
        """
    @transformation.setter
    def transformation(self, arg1: mrpt.poses.CPose3D) -> None:
        ...
def se2_l2(arg0: TMatchingPairList) -> tuple:
    """
    Least-squares SE(2) estimation. Returns (ok, mrpt::math::TPose2D)
    """
def se3_l2_robust(arg0: TMatchingPairList, arg1: TSE3RobustParams) -> tuple:
    """
    Robust SE(3) estimation using RANSAC. Returns (ok, TSE3RobustResult)
    """
