"""
Python bindings for mrpt::slam — SLAM and localization algorithms
"""
from __future__ import annotations
import mrpt.bayes
import mrpt.config
import mrpt.maps
import mrpt.math
import mrpt.obs
import mrpt.poses
import mrpt.viz
import typing
__all__: list[str] = ['CICP', 'CICPOptions', 'CMetricMapBuilder', 'CMetricMapBuilderICP', 'CMetricMapBuilderICPOptions', 'CMetricMapBuilderRBPF', 'CMonteCarloLocalization2D', 'CMonteCarloLocalization3D', 'TICPAlgorithm', 'TICPCovarianceMethod', 'TICPReturnInfo', 'TKLDParams', 'TMonteCarloLocalizationParams', 'TPredictionParams', 'icpClassic', 'icpCovFiniteDifferences', 'icpCovLinealMSE', 'icpLevenbergMarquardt']
class TICPAlgorithm:
    """
    Members:
    
      icpClassic
    
      icpLevenbergMarquardt
    """
    __members__: typing.ClassVar[dict[str, TICPAlgorithm]]
    icpClassic: typing.ClassVar[TICPAlgorithm]
    icpLevenbergMarquardt: typing.ClassVar[TICPAlgorithm]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class TICPCovarianceMethod:
    """
    Members:
    
      icpCovLinealMSE
    
      icpCovFiniteDifferences
    """
    __members__: typing.ClassVar[dict[str, TICPCovarianceMethod]]
    icpCovFiniteDifferences: typing.ClassVar[TICPCovarianceMethod]
    icpCovLinealMSE: typing.ClassVar[TICPCovarianceMethod]
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: int) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: int) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
class TICPReturnInfo:
    """
    The ICP algorithm return information.
    """
    goodness: float
    nIterations: int
    quality: float
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
class CICPOptions(mrpt.config.CLoadableOptions):
    """
    The ICP algorithm configuration data.
    """
    ALFA: float
    ICP_algorithm: TICPAlgorithm
    ICP_covariance_method: TICPCovarianceMethod
    corresponding_points_decimation: int
    doRANSAC: bool
    maxIterations: int
    skip_cov_calculation: bool
    skip_quality_calculation: bool
    smallestThresholdDist: float
    thresholdAng: float
    thresholdDist: float
    def __init__(self) -> None:
        """
        Default constructor.
        """
class CICP:
    """
    Several implementations of ICP (Iterative closest point) algorithms for aligning two point maps or a point map wrt a grid map.
    """
    options: CICPOptions
    def AlignPDF(self, m1: mrpt.maps.CMetricMap, m2: mrpt.maps.CMetricMap, initialEstimationPDF: mrpt.poses.CPosePDFGaussian) -> tuple:
        """
        Align two maps. Returns (CPosePDF, TICPReturnInfo) tuple.
        """
    @typing.overload
    def __init__(self) -> None:
        """
        Constructor with the default options.
        """
    @typing.overload
    def __init__(self, options: CICPOptions) -> None:
        """
        Constructor that directly set the ICP params from a given struct.
        """
    def __repr__(self) -> str:
        ...
class CMetricMapBuilder:
    """
    Base class of the SLAM map builders.
    """
    def getCurrentPoseEstimation(self) -> mrpt.poses.CPose3DPDF:
        """
        Returns a copy of the current best pose estimation as a pose PDF.
        """
    def getCurrentlyBuiltMapSize(self) -> int:
        """
        Returns just how many sensory-frames are stored in the currently build map.
        """
    @typing.overload
    def initialize(self) -> None:
        """
        Initialize the builder with an empty map.
        """
    @typing.overload
    def initialize(self, initialMap: mrpt.obs.CSimpleMap) -> None:
        """
        Initialize the builder with a given initial map.
        """
    def processActionObservation(self, action: mrpt.obs.CActionCollection, sf: mrpt.obs.CSensoryFrame) -> None:
        """
        Updates the map and pose estimate with a new action and sensory frame.
        """
    def saveCurrentMapToFile(self, fileName: str, compressGZ: bool = True) -> None:
        """
        Saves the current map (a CSimpleMap) to a .simplemap file.
        """
class CMetricMapBuilderICPOptions(mrpt.config.CLoadableOptions):
    """
    Options of CMetricMapBuilderICP.
    """
    insertionAngDistance: float
    insertionLinDistance: float
    localizationAngDistance: float
    localizationLinDistance: float
    matchAgainstTheGrid: bool
    minICPgoodnessToAccept: float
    @typing.overload
    def loadFromConfigFile(self, iniContent: str, section: str) -> None:
        """
        Load options (including mapInitializers) from an INI-format string.
        """
    @typing.overload
    def loadFromConfigFile(self, source: mrpt.config.CConfigFileBase, section: str) -> None:
        """
        Load options (including mapInitializers) from a config file.
        """
class CMetricMapBuilderICP(CMetricMapBuilder):
    """
    A class for very simple 2D SLAM based on ICP. This is a non-probabilistic pose tracking algorithm.
    """
    ICP_options: CMetricMapBuilderICPOptions
    ICP_params: CICPOptions
    def __init__(self) -> None:
        """
        Default constructor: set ICP_options, then call initialize().
        """
    def __repr__(self) -> str:
        ...
    def getCurrentMapPoints(self) -> tuple:
        """
        Returns (xs, ys) float lists of current point-map coordinates.
        """
    def getCurrentPoseEstimation(self) -> mrpt.poses.CPose3DPDF:
        """
        Returns a copy of the current best pose estimation as a pose PDF.
        """
    def getCurrentlyBuiltMapSize(self) -> int:
        """
        Returns just how many sensory-frames are stored in the currently build map.
        """
    @typing.overload
    def initialize(self) -> None:
        """
        Starts with an empty map.
        """
    @typing.overload
    def initialize(self, initialMap: mrpt.obs.CSimpleMap) -> None:
        """
        Starts from a given initial map.
        """
    def processActionObservation(self, action: mrpt.obs.CActionCollection, sf: mrpt.obs.CSensoryFrame) -> None:
        """
        Process action+sensoryframe pair (classic API).
        """
    def processObservation(self, obs: mrpt.obs.CObservation) -> None:
        """
        Process a single observation (new-style API).
        """
    def saveCurrentMapToFile(self, fileName: str, compressGZ: bool = True) -> None:
        """
        Saves the current map (a CSimpleMap) to a .simplemap file.
        """
    def useSimplePointsMap(self) -> None:
        """
        Configure the builder to use a single CSimplePointsMap. Call before initialize().
        """
class TKLDParams(mrpt.config.CLoadableOptions):
    """
    Option set for KLD algorithm.
    """
    KLD_binSize_PHI: float
    KLD_binSize_XY: float
    KLD_delta: float
    KLD_epsilon: float
    KLD_maxSampleSize: int
    KLD_minSampleSize: int
    KLD_minSamplesPerBin: float
    def __init__(self) -> None:
        """
        Default constructor.
        """
class TMonteCarloLocalizationParams:
    """
    Parameters of the prediction and update stages of Monte Carlo localization.
    """
    KLD_params: TKLDParams
    def __init__(self) -> None:
        """
        Default constructor.
        """
    @property
    def metricMap(self) -> mrpt.maps.CMetricMap:
        """
        The map used to evaluate observation likelihoods (e.g. a CMultiMetricMap)
        """
    @metricMap.setter
    def metricMap(self, arg1: mrpt.maps.CMetricMap) -> None:
        ...
    @property
    def metricMaps(self) -> list[mrpt.maps.CMetricMap]:
        """
        Alternative to metricMap: one map per particle (rarely used)
        """
    @metricMaps.setter
    def metricMaps(self, arg1: list[mrpt.maps.CMetricMap]) -> None:
        ...
class CMonteCarloLocalization2D(mrpt.poses.CPosePDFParticles):
    """
    Particle filter for 2D robot localization (x, y, phi) on a known map.
    """
    options: TMonteCarloLocalizationParams
    def __init__(self, M: int = 1) -> None:
        """
        Creates a filter with M particles
        """
    def __repr__(self) -> str:
        ...
    def getVisualization(self) -> mrpt.viz.CSetOfObjects:
        """
        Returns a 3D representation of the particles
        """
    def resetUniformFreeSpace(self, theMap: mrpt.maps.COccupancyGridMap2D, freeCellsThreshold: float = 0.7, particlesCount: int = -1, x_min: float = -10000000000.0, x_max: float = 10000000000.0, y_min: float = -10000000000.0, y_max: float = 10000000000.0, phi_min: float = -3.141592653589793, phi_max: float = 3.141592653589793) -> None:
        """
        Spreads particles uniformly over the free space of an occupancy grid (global localization)
        """
class CMonteCarloLocalization3D(mrpt.poses.CPose3DPDFParticles):
    """
    Particle filter for 3D robot localization on a known map.
    """
    options: TMonteCarloLocalizationParams
    def __init__(self, M: int = 1) -> None:
        """
        Creates a filter with M particles
        """
    def __repr__(self) -> str:
        ...
    def getVisualization(self) -> mrpt.viz.CSetOfObjects:
        """
        Returns a 3D representation of the particles
        """
class TPredictionParams(mrpt.config.CLoadableOptions):
    """
    Parameters of the prediction and update stages of RBPF-SLAM.
    """
    ICPGlobalAlign_MinQuality: float
    KLD_params: TKLDParams
    icp_params: CICPOptions
    pfOptimalProposal_mapSelection: int
    def __init__(self) -> None:
        """
        Default constructor.
        """
class CMetricMapBuilderRBPF(CMetricMapBuilder):
    """
    This class implements a Rao-Blackwelized Particle Filter (RBPF) approach to map building (SLAM).
    """
    class TConstructionOptions(mrpt.config.CLoadableOptions):
        """
        Options of CMetricMapBuilderRBPF.
        """
        PF_options: mrpt.bayes.TParticleFilterOptions
        insertionAngDistance: float
        insertionLinDistance: float
        localizeAngDistance: float
        localizeLinDistance: float
        mapsInitializers: mrpt.obs.TSetOfMetricMapInitializers
        predictionOptions: TPredictionParams
        def __init__(self) -> None:
            """
            Default constructor.
            """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: set the options, then call initialize().
        """
    @typing.overload
    def __init__(self, options: CMetricMapBuilderRBPF.TConstructionOptions) -> None:
        """
        Builds the RBPF-SLAM map builder from its options.
        """
    def clear(self) -> None:
        """
        Clears all maps and resets the filter
        """
    def getCurrentJointEntropy(self) -> float:
        """
        Returns the joint entropy of the map and path estimate.
        """
    def getCurrentMostLikelyPath(self) -> list[mrpt.math.TPose3D]:
        """
        Returns the robot path of the most likely particle, as a list of TPose3D
        """
    def getCurrentPoseEstimation(self) -> mrpt.poses.CPose3DPDF:
        """
        Returns the current robot pose estimation (a CPose3DPDF)
        """
    def getCurrentlyBuiltMap(self) -> mrpt.obs.CSimpleMap:
        """
        Returns the keyframes of the most likely particle as a CSimpleMap
        """
    def getCurrentlyBuiltMapSize(self) -> int:
        """
        Returns just how many sensory-frames are stored in the currently build map.
        """
    def getCurrentlyBuiltMetricMap(self) -> mrpt.maps.CMultiMetricMap:
        """
        Returns a copy of the map of the most likely particle (the particles are replaced as the filter runs, so a reference would not stay valid)
        """
    def initialize(self, initialMap: mrpt.obs.CSimpleMap = ...) -> None:
        """
        Resets the filter, optionally starting from a given map
        """
    def processActionObservation(self, action: mrpt.obs.CActionCollection, sf: mrpt.obs.CSensoryFrame) -> None:
        """
        Processes one (action, sensory frame) pair
        """
    def saveCurrentPathEstimationToTextFile(self, fileName: str) -> None:
        """
        A logging utility: saves the current path estimation for each particle in a text file (a row per particle, each 3-column-entry is a set [x,y,phi], respectively).
        """
icpClassic: TICPAlgorithm
icpCovFiniteDifferences: TICPCovarianceMethod
icpCovLinealMSE: TICPCovarianceMethod
icpLevenbergMarquardt: TICPAlgorithm
