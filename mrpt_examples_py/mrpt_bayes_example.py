#!/usr/bin/env python3
"""
mrpt_bayes_example.py — particle filter configuration with mrpt.bayes.

Demonstrates:
  - TParticleFilterOptions: configure algorithm, resampling, ESS threshold
  - TParticleFilterAlgorithm / TParticleResamplingAlgorithm enums
  - CParticleFilter: inspect and modify the filter object

NOTE: CParticleFilter.executeOn() requires a C++ CParticleFilterCapable PDF
object which cannot be constructed from pure Python.  For a full RBPF SLAM
example see rbpf_slam.py, which drives CMetricMapBuilderRBPF from C++.
This example focuses on what CAN be done directly: configuring the filter
parameters before passing them to a C++ algorithm.
"""

from mrpt.bayes import (
    CParticleFilter,
    TParticleFilterOptions,
    TParticleFilterAlgorithm,
    TParticleResamplingAlgorithm,
    TParticleFilterStats,
)

# ---------------------------------------------------------------------------
# Enumerations
# ---------------------------------------------------------------------------
print("TParticleFilterAlgorithm values:")
for name in ["pfStandardProposal", "pfAuxiliaryPFStandard",
             "pfOptimalProposal", "pfAuxiliaryPFOptimal"]:
    val = getattr(TParticleFilterAlgorithm, name)
    print(f"  {name} = {int(val)}")

print("\nTParticleResamplingAlgorithm values:")
for name in ["prMultinomial", "prResidual", "prStratified", "prSystematic"]:
    val = getattr(TParticleResamplingAlgorithm, name)
    print(f"  {name} = {int(val)}")

# ---------------------------------------------------------------------------
# CParticleFilter — configuration
# ---------------------------------------------------------------------------
pf = CParticleFilter()
print(f"\nDefault CParticleFilter: {pf}")

# Tune options
pf.options.BETA           = 0.75         # resample when ESS drops below 75%
pf.options.sampleSize     = 200          # number of particles
pf.options.PF_algorithm   = TParticleFilterAlgorithm.pfStandardProposal
pf.options.resamplingMethod = TParticleResamplingAlgorithm.prStratified

print(f"After tuning: BETA={pf.options.BETA}, sampleSize={pf.options.sampleSize}")
print(f"  algorithm = {pf.options.PF_algorithm}")
print(f"  resampling = {pf.options.resamplingMethod}")
print(f"  {pf.options}")

# ---------------------------------------------------------------------------
# TParticleFilterStats — result statistics container
# ---------------------------------------------------------------------------
stats = TParticleFilterStats()
print(f"\nTParticleFilterStats (empty): ESS={stats.ESS_beforeResample:.4f}")
# In a real run, stats would be filled by executeOn()
stats.ESS_beforeResample = 0.42
print(f"After setting: {stats}")
