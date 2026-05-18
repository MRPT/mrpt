#!/usr/bin/env python3
"""Smoke tests for mrpt.bayes Python bindings."""
import sys

try:
    from mrpt.bayes import (
        CParticleFilter, TParticleFilterOptions,
        TParticleFilterAlgorithm, TParticleResamplingAlgorithm,
        CParticleFilterCapable,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.bayes bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.bayes import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("TParticleFilterOptions")
opts = TParticleFilterOptions()
check("default sampleSize > 0", opts.sampleSize > 0, f"got {opts.sampleSize}")
opts.sampleSize = 500
check("sampleSize setter", opts.sampleSize == 500)

print("TParticleFilterAlgorithm enum")
check("StandardProposal exists",
      hasattr(TParticleFilterAlgorithm, "StandardProposal"))
check("pfStandardProposal alias exists",
      hasattr(TParticleFilterAlgorithm, "pfStandardProposal"))

print("TParticleResamplingAlgorithm enum")
check("Multinomial exists",
      hasattr(TParticleResamplingAlgorithm, "Multinomial"))
check("prMultinomial alias exists",
      hasattr(TParticleResamplingAlgorithm, "prMultinomial"))

print("CParticleFilter")
pf = CParticleFilter()
check("CParticleFilter created", pf is not None)

print("CParticleFilterCapable.logWeightsToLinear")
import math
log_w = [-1.0, -2.0, -0.5]
lin_w = CParticleFilterCapable.logWeightsToLinear(log_w)
check("logWeightsToLinear sum=1", abs(sum(lin_w) - 1.0) < 1e-10, f"sum={sum(lin_w)}")
check("logWeightsToLinear max at index 2", lin_w.index(max(lin_w)) == 2)

print("CParticleFilterCapable.computeResampling")
log_w_unif = [0.0] * 10
idx = CParticleFilterCapable.computeResampling(
    TParticleResamplingAlgorithm.Multinomial, log_w_unif)
check("computeResampling returns correct count", len(idx) == 10, f"got {len(idx)}")
check("computeResampling indexes in range", all(0 <= i < 10 for i in idx))

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
