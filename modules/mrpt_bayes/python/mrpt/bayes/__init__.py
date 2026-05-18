# mrpt/bayes/__init__.py

"""
mrpt-bayes Python API — particle filter configuration and Bayesian estimation.

The CParticleFilter class configures and drives a particle filter; the actual
PDF being estimated must implement CParticleFilterCapable (in C++) or be
driven via the mrpt.slam RBPF classes from Python.
"""

from . import _bindings as _b

# Enumerations
TParticleFilterAlgorithm      = _b.TParticleFilterAlgorithm
TParticleResamplingAlgorithm  = _b.TParticleResamplingAlgorithm

# Configuration / statistics structs
TParticleFilterOptions = _b.TParticleFilterOptions
TParticleFilterStats   = _b.TParticleFilterStats

# Main classes
CParticleFilter        = _b.CParticleFilter
CParticleFilterCapable = _b.CParticleFilterCapable

__all__ = [
    "TParticleFilterAlgorithm",
    "TParticleResamplingAlgorithm",
    "TParticleFilterOptions",
    "TParticleFilterStats",
    "CParticleFilter",
    "CParticleFilterCapable",
]
