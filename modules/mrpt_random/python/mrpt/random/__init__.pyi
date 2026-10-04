"""
mrpt.random — Random number generation utilities.

Provides:
  - CRandomGenerator : Mersenne Twister PRNG with Gaussian/uniform sampling
  - getRandomGenerator() : Global singleton accessor
  - Randomize(seed)      : Seed the global generator

Example::

    import mrpt.random as rng
    g = rng.CRandomGenerator(42)
    x = g.drawGaussian1D(mean=0.0, std=1.0)
    arr = g.drawGaussianArray(1000, mean=0.0, std=1.0)  # numpy array
    xy = g.drawGaussianMultivariateMany(100, [[1.0, 0.5], [0.5, 2.0]])  # (100, 2)
"""
from __future__ import annotations
from mrpt.random._bindings import CRandomGenerator as CRandomGenerator
from mrpt.random._bindings import Randomize as Randomize
from mrpt.random._bindings import getRandomGenerator as getRandomGenerator
from . import _bindings
__all__: list = ['CRandomGenerator', 'getRandomGenerator', 'Randomize']
def _drawGaussianMultivariate(self, cov, mean = None):
    """
    Draw one sample from N(mean, cov) as a 1D numpy array (zero mean if None).
    """
def _drawGaussianMultivariateMany(self, n, cov, mean = None):
    """
    Draw n samples from N(mean, cov) as the rows of an (n, dim) numpy array.
    
        Zero mean if `mean` is None. `cov` must be symmetric and positive
        semi-definite; an eigen-decomposition (not Cholesky) is used, so singular
        covariances are accepted, like the C++
        CRandomGenerator::drawGaussianMultivariate().
        
    """
