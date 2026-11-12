"""
Python bindings for mrpt_tfest (Transformation Estimation)
"""
from __future__ import annotations
import mrpt.math
import mrpt.poses
import typing
__all__: list[str] = ['TMatchingPair', 'TMatchingPairList', 'TMatchingPairList_d', 'TMatchingPair_d', 'TPotentialMatch', 'TSE2RobustParams', 'TSE2RobustResult', 'TSE3RobustParams', 'TSE3RobustResult', 'se2_l2', 'se2_l2_robust', 'se2_l2_with_covariance', 'se3_l2', 'se3_l2_robust']
class TMatchingPair:
    """
    A pair of corresponding points (float coordinates) in two point sets.
    """
    errorSquareAfterTransformation: float
    globalIdx: int
    global_pt: mrpt.math.TPoint3Df
    localIdx: int
    local_pt: mrpt.math.TPoint3Df
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, globalIdx: int, localIdx: int, global_pt: mrpt.math.TPoint3Df, local_pt: mrpt.math.TPoint3Df) -> None:
        """
        Builds the pair from the indices and coordinates of the global and local points.
        """
    @typing.overload
    def __init__(self, globalIdx: int, localIdx: int, global_x: float, global_y: float, global_z: float, local_x: float, local_y: float, local_z: float) -> None:
        """
        Builds the pair from the indices and coordinates of the global and local points.
        """
    def __repr__(self) -> str:
        ...
class TMatchingPair_d:
    """
    A pair of corresponding points (double coordinates) in two point sets.
    """
    errorSquareAfterTransformation: float
    globalIdx: int
    global_pt: mrpt.math.TPoint3D
    localIdx: int
    local_pt: mrpt.math.TPoint3D
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @typing.overload
    def __init__(self, globalIdx: int, localIdx: int, global_pt: mrpt.math.TPoint3D, local_pt: mrpt.math.TPoint3D) -> None:
        """
        Builds the pair from the indices and coordinates of the global and local points.
        """
    @typing.overload
    def __init__(self, globalIdx: int, localIdx: int, global_x: float, global_y: float, global_z: float, local_x: float, local_y: float, local_z: float) -> None:
        """
        Builds the pair from the indices and coordinates of the global and local points.
        """
    def __repr__(self) -> str:
        ...
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
class TSE2RobustParams:
    """
    Parameters of se2_l2_robust().
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def max_rmse_to_end(self) -> float:
        """
        Stops when a solution has an RMSE below this value. 0 (default) means 2*normalizationStd.
        """
    @max_rmse_to_end.setter
    def max_rmse_to_end(self, arg0: float) -> None:
        ...
    @property
    def probability_find_good_model(self) -> float:
        """
        Probability of finding a good model, for the adaptive number of iterations (default: 0.999).
        """
    @probability_find_good_model.setter
    def probability_find_good_model(self, arg0: float) -> None:
        ...
    @property
    def ransac_algorithmForLandmarks(self) -> bool:
        """
        Use the Mahalanobis distance (true, default) or the Euclidean distance (false).
        """
    @ransac_algorithmForLandmarks.setter
    def ransac_algorithmForLandmarks(self, arg0: bool) -> None:
        ...
    @property
    def ransac_fuseByCorrsMatch(self) -> bool:
        """
        If true (default), modes are fused when their subsets of pairs match; otherwise, when their means are closer than ransac_fuseMaxDiffXY and ransac_fuseMaxDiffPhi.
        """
    @ransac_fuseByCorrsMatch.setter
    def ransac_fuseByCorrsMatch(self, arg0: bool) -> None:
        ...
    @property
    def ransac_fuseMaxDiffPhi(self) -> float:
        """
        Maximum difference in phi to fuse two modes, in radians (default: 0.1 deg).
        """
    @ransac_fuseMaxDiffPhi.setter
    def ransac_fuseMaxDiffPhi(self, arg0: float) -> None:
        ...
    @property
    def ransac_fuseMaxDiffXY(self) -> float:
        """
        Maximum difference in x and y to fuse two modes (default: 0.01).
        """
    @ransac_fuseMaxDiffXY.setter
    def ransac_fuseMaxDiffXY(self, arg0: float) -> None:
        ...
    @property
    def ransac_mahalanobisDistanceThreshold(self) -> float:
        """
        Threshold to accept a pair into a consensus set (default: 3.0).
        """
    @ransac_mahalanobisDistanceThreshold.setter
    def ransac_mahalanobisDistanceThreshold(self, arg0: float) -> None:
        ...
    @property
    def ransac_maxSetSize(self) -> int:
        """
        Number of pairs tested for consensus in each iteration (default: 20). Set it to the number of correspondences to test all of them.
        """
    @ransac_maxSetSize.setter
    def ransac_maxSetSize(self, arg0: int) -> None:
        ...
    @property
    def ransac_minSetSize(self) -> int:
        """
        Minimum size of a consensus set (default: 3).
        """
    @ransac_minSetSize.setter
    def ransac_minSetSize(self, arg0: int) -> None:
        ...
    @property
    def ransac_min_nSimulations(self) -> int:
        """
        Minimum number of iterations with the adaptive method (default: 1500).
        """
    @ransac_min_nSimulations.setter
    def ransac_min_nSimulations(self, arg0: int) -> None:
        ...
    @property
    def ransac_nSimulations(self) -> int:
        """
        Number of iterations, or 0 (default) for an adaptive number, from probability_find_good_model.
        """
    @ransac_nSimulations.setter
    def ransac_nSimulations(self, arg0: int) -> None:
        ...
    @property
    def user_individual_compat_callback(self) -> typing.Callable[[TPotentialMatch], bool]:
        """
        Optional function(TPotentialMatch) -> bool: return True to accept a candidate pairing, False to reject it (None: accept all)
        """
    @user_individual_compat_callback.setter
    def user_individual_compat_callback(self, arg0: typing.Callable[[TPotentialMatch], bool]) -> None:
        ...
    @property
    def verbose(self) -> bool:
        """
        Prints debug information.
        """
    @verbose.setter
    def verbose(self, arg0: bool) -> None:
        ...
class TSE2RobustResult:
    """
    Result of se2_l2_robust().
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def largestSubSet(self) -> TMatchingPairList:
        """
        The consensus subset of pairs with the lowest RMSE.
        """
    @largestSubSet.setter
    def largestSubSet(self, arg0: TMatchingPairList) -> None:
        ...
    @property
    def ransac_iters(self) -> int:
        """
        Number of iterations executed.
        """
    @ransac_iters.setter
    def ransac_iters(self, arg0: int) -> None:
        ...
    @property
    def transformation(self) -> mrpt.poses.CPosePDFSOG:
        """
        The estimated transformations, as a sum of Gaussians (mrpt.poses.CPosePDFSOG).
        """
    @transformation.setter
    def transformation(self, arg0: mrpt.poses.CPosePDFSOG) -> None:
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
    def forceScaleToUnity(self) -> bool:
        """
        Estimates a rigid transformation, with scale 1 (default: true).
        """
    @forceScaleToUnity.setter
    def forceScaleToUnity(self, arg0: bool) -> None:
        ...
    @property
    def ransac_threshold_ang(self) -> float:
        """
        Maximum rotation difference for a solution to match a candidate, in radians (default: 1 deg).
        """
    @ransac_threshold_ang.setter
    def ransac_threshold_ang(self, arg0: float) -> None:
        ...
    @property
    def ransac_threshold_scale(self) -> float:
        """
        Maximum scale difference for a solution to match a candidate (default: 0.03).
        """
    @ransac_threshold_scale.setter
    def ransac_threshold_scale(self, arg0: float) -> None:
        ...
    @property
    def user_individual_compat_callback(self) -> typing.Callable[[TPotentialMatch], bool]:
        """
        Optional function(TPotentialMatch) -> bool: return True to accept a candidate pairing, False to reject it (None: accept all)
        """
    @user_individual_compat_callback.setter
    def user_individual_compat_callback(self, arg0: typing.Callable[[TPotentialMatch], bool]) -> None:
        ...
    @property
    def verbose(self) -> bool:
        """
        Prints debug information.
        """
    @verbose.setter
    def verbose(self, arg0: bool) -> None:
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
def se2_l2_robust(in_correspondences: TMatchingPairList, in_normalizationStd: float, in_ransac_params: TSE2RobustParams) -> tuple:
    """
    Robust SE(2) estimation using RANSAC, returning a sum of Gaussians. in_normalizationStd is the standard deviation of the point coordinates. Returns (ok, TSE2RobustResult).
    """
def se2_l2_with_covariance(correspondences: TMatchingPairList) -> tuple:
    """
    Least-squares SE(2) estimation, also returning its normalized 3x3 covariance, which must be multiplied by the variance of the point coordinates. Returns (ok, mrpt.math.TPose2D, mrpt.math.CMatrixDouble33). ok is false for less than two pairs, or if the rotation is undefined.
    """
@typing.overload
def se3_l2(in_correspondences: TMatchingPairList, forceScaleToUnity: bool = False) -> tuple:
    """
    Least-squares SE(3) estimation (Horn's method). Returns (ok, mrpt.poses.CPose3D, scale).
    """
@typing.overload
def se3_l2(in_correspondences: TMatchingPairList_d, forceScaleToUnity: bool = False) -> tuple:
    """
    Least-squares SE(3) estimation (Horn's method), for double precision pairs. Returns (ok, mrpt.poses.CPose3D, scale).
    """
@typing.overload
def se3_l2(in_points_this: list[mrpt.math.TPoint3D], in_points_other: list[mrpt.math.TPoint3D], forceScaleToUnity: bool = False) -> tuple:
    """
    Least-squares SE(3) estimation (Horn's method) from two lists of corresponding points of the same length. Returns (ok, mrpt.poses.CPose3D, scale).
    """
def se3_l2_robust(arg0: TMatchingPairList, arg1: TSE3RobustParams) -> tuple:
    """
    Robust SE(3) estimation using RANSAC. Returns (ok, TSE3RobustResult)
    """
