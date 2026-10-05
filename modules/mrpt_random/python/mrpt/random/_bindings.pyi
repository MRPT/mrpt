"""
Python bindings for mrpt::random — random number generators
"""
from __future__ import annotations
import numpy
import typing
__all__: list[str] = ['CRandomGenerator', 'Randomize', 'getRandomGenerator']
class CRandomGenerator:
    """
    A thread-safe pseudo random number generator, based on an internal MT19937 randomness generator.
    """
    @typing.overload
    def __init__(self) -> None:
        """
        Default constructor: initialize random seed based on current time.
        """
    @typing.overload
    def __init__(self, seed: int) -> None:
        """
        Constructor for providing a custom random seed to initialize the PRNG.
        """
    def __repr__(self) -> str:
        ...
    def drawGaussian1D(self, mean: float, std: float) -> float:
        """
        Draw a sample from N(mean, std)
        """
    def drawGaussian1D_normalized(self) -> float:
        """
        Draw a sample from N(0,1)
        """
    def drawGaussianArray(self, n: int, mean: float = 0.0, std: float = 1.0) -> numpy.ndarray:
        """
        Draw n Gaussian samples as a 1D float64 numpy array
        """
    def drawGaussianMultivariate(self, cov, mean = None):
        """
        Draw one sample from N(mean, cov) as a 1D numpy array (zero mean if None).
        """
    def drawGaussianMultivariateMany(self, n, cov, mean = None):
        """
        Draw n samples from N(mean, cov) as the rows of an (n, dim) numpy array.
        
            Zero mean if `mean` is None. `cov` must be symmetric and positive
            semi-definite; an eigen-decomposition (not Cholesky) is used, so singular
            covariances are accepted, like the C++
            CRandomGenerator::drawGaussianMultivariate().
            
        """
    def drawUniform(self, min: float, max: float) -> float:
        """
        Draw a uniform double from [min, max)
        """
    def drawUniform32bit(self) -> int:
        """
        Generate a uniformly distributed pseudo-random number using the MT19937 algorithm, in the whole range of 32-bit integers.
        """
    def drawUniform64bit(self) -> int:
        """
        Returns a uniformly distributed pseudo-random number by joining two 32bit numbers from drawUniform32bit()
        """
    def drawUniformArray(self, n: int, min: float = 0.0, max: float = 1.0) -> numpy.ndarray:
        """
        Draw n uniform samples as a 1D float64 numpy array
        """
    @typing.overload
    def randomize(self, seed: int) -> None:
        """
        Initialize the PRNG from the given seed
        """
    @typing.overload
    def randomize(self) -> None:
        """
        Initialize the PRNG from the current time (non-deterministic)
        """
@typing.overload
def Randomize(seed: int) -> None:
    """
    Seed the global random generator
    """
@typing.overload
def Randomize() -> None:
    """
    Seed the global random generator from the current time
    """
def getRandomGenerator() -> CRandomGenerator:
    """
    Returns the global MRPT random generator singleton
    """
