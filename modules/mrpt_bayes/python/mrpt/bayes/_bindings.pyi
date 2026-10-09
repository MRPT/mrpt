"""
Python bindings for mrpt::bayes — particle filter and Bayesian estimation
"""
from __future__ import annotations
import mrpt.config
import typing
__all__: list[str] = ['AuxiliaryPFOptimal', 'AuxiliaryPFStandard', 'CParticleFilter', 'CParticleFilterCapable', 'Multinomial', 'OptimalProposal', 'Residual', 'StandardProposal', 'Stratified', 'Systematic', 'TParticleFilterAlgorithm', 'TParticleFilterOptions', 'TParticleFilterStats', 'TParticleResamplingAlgorithm', 'pfAuxiliaryPFOptimal', 'pfAuxiliaryPFStandard', 'pfOptimalProposal', 'pfStandardProposal', 'prMultinomial', 'prResidual', 'prStratified', 'prSystematic']
class TParticleFilterAlgorithm:
    """
    Members:
    
      StandardProposal
    
      AuxiliaryPFStandard
    
      OptimalProposal
    
      AuxiliaryPFOptimal
    
      pfStandardProposal
    
      pfAuxiliaryPFStandard
    
      pfOptimalProposal
    
      pfAuxiliaryPFOptimal
    """
    AuxiliaryPFOptimal: typing.ClassVar[TParticleFilterAlgorithm]
    AuxiliaryPFStandard: typing.ClassVar[TParticleFilterAlgorithm]
    OptimalProposal: typing.ClassVar[TParticleFilterAlgorithm]
    StandardProposal: typing.ClassVar[TParticleFilterAlgorithm]
    __members__: typing.ClassVar[dict[str, TParticleFilterAlgorithm]]
    pfAuxiliaryPFOptimal: typing.ClassVar[TParticleFilterAlgorithm]
    pfAuxiliaryPFStandard: typing.ClassVar[TParticleFilterAlgorithm]
    pfOptimalProposal: typing.ClassVar[TParticleFilterAlgorithm]
    pfStandardProposal: typing.ClassVar[TParticleFilterAlgorithm]
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
class TParticleResamplingAlgorithm:
    """
    Members:
    
      Multinomial
    
      Residual
    
      Stratified
    
      Systematic
    
      prMultinomial
    
      prResidual
    
      prStratified
    
      prSystematic
    """
    Multinomial: typing.ClassVar[TParticleResamplingAlgorithm]
    Residual: typing.ClassVar[TParticleResamplingAlgorithm]
    Stratified: typing.ClassVar[TParticleResamplingAlgorithm]
    Systematic: typing.ClassVar[TParticleResamplingAlgorithm]
    __members__: typing.ClassVar[dict[str, TParticleResamplingAlgorithm]]
    prMultinomial: typing.ClassVar[TParticleResamplingAlgorithm]
    prResidual: typing.ClassVar[TParticleResamplingAlgorithm]
    prStratified: typing.ClassVar[TParticleResamplingAlgorithm]
    prSystematic: typing.ClassVar[TParticleResamplingAlgorithm]
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
class TParticleFilterOptions(mrpt.config.CLoadableOptions):
    """
    The configuration of a particle filter algorithm and resampling parameters.
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    @property
    def BETA(self) -> float:
        """
        Resampling threshold: resample when ESS < BETA (default=0.5)
        """
    @BETA.setter
    def BETA(self, arg0: float) -> None:
        ...
    @property
    def PF_algorithm(self) -> TParticleFilterAlgorithm:
        """
        The particle filter algorithm to use
        """
    @PF_algorithm.setter
    def PF_algorithm(self, arg0: TParticleFilterAlgorithm) -> None:
        ...
    @property
    def adaptiveSampleSize(self) -> bool:
        """
        If true, enable adaptive number of particles
        """
    @adaptiveSampleSize.setter
    def adaptiveSampleSize(self, arg0: bool) -> None:
        ...
    @property
    def resamplingMethod(self) -> TParticleResamplingAlgorithm:
        """
        The resampling scheme (default=prMultinomial)
        """
    @resamplingMethod.setter
    def resamplingMethod(self, arg0: TParticleResamplingAlgorithm) -> None:
        ...
    @property
    def sampleSize(self) -> int:
        """
        Initial number of particles (relevant for adaptiveSampleSize=false)
        """
    @sampleSize.setter
    def sampleSize(self, arg0: int) -> None:
        ...
class TParticleFilterStats:
    """
    Statistics returned by CParticleFilter.executeOn().
    """
    def __init__(self) -> None:
        """
        Default constructor.
        """
    def __repr__(self) -> str:
        ...
    @property
    def ESS_beforeResample(self) -> float:
        """
        Effective sample size (ESS) before resampling step
        """
    @ESS_beforeResample.setter
    def ESS_beforeResample(self, arg0: float) -> None:
        ...
    @property
    def weightsVariance_beforeResample(self) -> float:
        """
        Weight variance before resampling
        """
    @weightsVariance_beforeResample.setter
    def weightsVariance_beforeResample(self, arg0: float) -> None:
        ...
class CParticleFilter:
    """
    Generic particle filter: runs one prediction/update/resampling step of a CParticleFilterCapable PDF with executeOn() (added by importing mrpt.slam).
    """
    def __init__(self) -> None:
        """
        Default constructor: set the options, then call executeOn() for each filter step (import mrpt.slam first, which adds that method).
        """
    def __repr__(self) -> str:
        ...
    @property
    def options(self) -> TParticleFilterOptions:
        """
        Algorithm options (TParticleFilterOptions)
        """
    @options.setter
    def options(self, arg0: TParticleFilterOptions) -> None:
        ...
class CParticleFilterCapable:
    """
    Interface of the particle-based PDFs that a CParticleFilter can run on.
    """
    @staticmethod
    def computeResampling(method: TParticleResamplingAlgorithm, log_weights: list[float], out_particle_count: int = 0) -> list[int]:
        """
        Compute resampling indexes from log-weights.
        Returns a list of particle indices after resampling.
        """
    @staticmethod
    def logWeightsToLinear(log_weights: list[float]) -> list[float]:
        """
        Convert log-weights to normalized linear weights (sum=1).
        """
    def ESS(self) -> float:
        """
        Effective sample size, in the range [0,1]
        """
    def getW(self, i: int) -> float:
        """
        Returns the log-weight of particle i
        """
    def normalizeWeights(self) -> float:
        """
        Normalizes the log-weights so the maximum is 0. Returns the max log-weight before normalizing.
        """
    def particlesCount(self) -> int:
        """
        Returns the number of particles.
        """
    def setW(self, i: int, w: float) -> None:
        """
        Sets the log-weight of particle i
        """
AuxiliaryPFOptimal: TParticleFilterAlgorithm
AuxiliaryPFStandard: TParticleFilterAlgorithm
Multinomial: TParticleResamplingAlgorithm
OptimalProposal: TParticleFilterAlgorithm
Residual: TParticleResamplingAlgorithm
StandardProposal: TParticleFilterAlgorithm
Stratified: TParticleResamplingAlgorithm
Systematic: TParticleResamplingAlgorithm
pfAuxiliaryPFOptimal: TParticleFilterAlgorithm
pfAuxiliaryPFStandard: TParticleFilterAlgorithm
pfOptimalProposal: TParticleFilterAlgorithm
pfStandardProposal: TParticleFilterAlgorithm
prMultinomial: TParticleResamplingAlgorithm
prResidual: TParticleResamplingAlgorithm
prStratified: TParticleResamplingAlgorithm
prSystematic: TParticleResamplingAlgorithm
