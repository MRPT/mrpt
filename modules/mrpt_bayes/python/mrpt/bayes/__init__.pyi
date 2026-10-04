"""
mrpt-bayes Python API — particle filter configuration and Bayesian estimation.

The CParticleFilter class configures and drives a particle filter; the actual
PDF being estimated must implement CParticleFilterCapable (in C++) or be
driven via the mrpt.slam RBPF classes from Python.
"""
from __future__ import annotations
import mrpt as mrpt
from mrpt.bayes._bindings import CParticleFilter as CParticleFilter
from mrpt.bayes._bindings import CParticleFilterCapable as CParticleFilterCapable
from mrpt.bayes._bindings import TParticleFilterAlgorithm as TParticleFilterAlgorithm
from mrpt.bayes._bindings import TParticleFilterOptions as TParticleFilterOptions
from mrpt.bayes._bindings import TParticleFilterStats as TParticleFilterStats
from mrpt.bayes._bindings import TParticleResamplingAlgorithm as TParticleResamplingAlgorithm
from . import _bindings
__all__: list = ['TParticleFilterAlgorithm', 'TParticleResamplingAlgorithm', 'TParticleFilterOptions', 'TParticleFilterStats', 'CParticleFilter', 'CParticleFilterCapable']
