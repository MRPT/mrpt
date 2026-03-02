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
"""

from mrpt.random._bindings import (
    CRandomGenerator,
    Randomize,
    getRandomGenerator,
)

__all__ = [
    "CRandomGenerator",
    "getRandomGenerator",
    "Randomize",
]
