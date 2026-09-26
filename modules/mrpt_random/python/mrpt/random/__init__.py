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

from mrpt.random._bindings import (
    CRandomGenerator,
    Randomize,
    getRandomGenerator,
)



def _drawGaussianMultivariateMany(self, n, cov, mean=None):
    """Draw n samples from N(mean, cov) as the rows of an (n, dim) numpy array.

    Zero mean if `mean` is None. Uses an eigen-decomposition (not Cholesky),
    so positive semi-definite covariances are accepted, like the C++
    CRandomGenerator::drawGaussianMultivariate().
    """
    import numpy as np

    cov = np.asarray(cov, dtype=np.float64)
    if cov.ndim != 2 or cov.shape[0] != cov.shape[1]:
        raise ValueError("cov must be a square matrix")
    dim = cov.shape[0]
    eig_vals, eig_vecs = np.linalg.eigh(cov)
    scaled = eig_vecs * np.sqrt(np.clip(eig_vals, 0.0, None))
    z = self.drawGaussianArray(n * dim, 0.0, 1.0).reshape(n, dim)
    samples = z @ scaled.T
    if mean is not None:
        mean = np.asarray(mean, dtype=np.float64)
        if mean.shape != (dim,):
            raise ValueError("mean and cov sizes do not match")
        samples += mean
    return samples


def _drawGaussianMultivariate(self, cov, mean=None):
    """Draw one sample from N(mean, cov) as a 1D numpy array (zero mean if None)."""
    return _drawGaussianMultivariateMany(self, 1, cov, mean)[0]


CRandomGenerator.drawGaussianMultivariateMany = _drawGaussianMultivariateMany
CRandomGenerator.drawGaussianMultivariate = _drawGaussianMultivariate

__all__ = [
    "CRandomGenerator",
    "getRandomGenerator",
    "Randomize",
]
