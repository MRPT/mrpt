#!/usr/bin/env python3
"""Smoke tests for mrpt.bayes Python bindings."""
import sys

try:
    from mrpt.bayes import (
        CParticleFilter, TParticleFilterOptions,
        TParticleFilterAlgorithm, TParticleResamplingAlgorithm,
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
check("pfStandardProposal exists",
      hasattr(TParticleFilterAlgorithm, "pfStandardProposal"))

print("TParticleResamplingAlgorithm enum")
check("prMultinomial exists",
      hasattr(TParticleResamplingAlgorithm, "prMultinomial"))

print("CParticleFilter")
pf = CParticleFilter()
check("CParticleFilter created", pf is not None)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
