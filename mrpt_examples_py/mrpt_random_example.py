#!/usr/bin/env python3
"""
mrpt_random_example.py — random number generation with mrpt.random.

Demonstrates:
  - CRandomGenerator: seeded deterministic generator
  - drawUniform / drawGaussian1D scalar draws
  - drawUniformArray / drawGaussianArray numpy output
  - Global getRandomGenerator() singleton
"""

import numpy as np
from mrpt.random import CRandomGenerator, getRandomGenerator, Randomize

# ---------------------------------------------------------------------------
# Seeded generator (deterministic)
# ---------------------------------------------------------------------------
rng = CRandomGenerator(42)

u = rng.drawUniform(0.0, 10.0)
g = rng.drawGaussian1D(5.0, 2.0)
print(f"Seeded drawUniform(0,10)     = {u:.4f}")
print(f"Seeded drawGaussian1D(5,2)   = {g:.4f}")

# Same seed → same result
rng2 = CRandomGenerator(42)
assert abs(rng2.drawUniform(0.0, 10.0) - u) < 1e-12, "Seeded RNG must be deterministic"
print("  determinism check ✓")

# ---------------------------------------------------------------------------
# Numpy array output
# ---------------------------------------------------------------------------
samples_u = rng.drawUniformArray(10000, 0.0, 1.0)
samples_g = rng.drawGaussianArray(10000, 0.0, 1.0)

print(f"\ndrawUniformArray(10000, 0, 1): mean={samples_u.mean():.4f}, std={samples_u.std():.4f}")
print(f"drawGaussianArray(10000, 0, 1): mean={samples_g.mean():.4f}, std={samples_g.std():.4f}")

assert abs(samples_u.mean() - 0.5) < 0.02,  "Uniform mean should be ~0.5"
assert abs(samples_g.mean() - 0.0) < 0.05,  "Gaussian mean should be ~0"
assert abs(samples_g.std()  - 1.0) < 0.05,  "Gaussian std should be ~1"
print("  statistics check ✓")

# ---------------------------------------------------------------------------
# Integer / 64-bit draws
# ---------------------------------------------------------------------------
u32 = rng.drawUniform32bit()
u64 = rng.drawUniform64bit()
print(f"\ndrawUniform32bit = {u32}")
print(f"drawUniform64bit = {u64}")

# ---------------------------------------------------------------------------
# Global singleton
# ---------------------------------------------------------------------------
global_rng = getRandomGenerator()
Randomize(123)
val = global_rng.drawUniform(0.0, 1.0)
print(f"\nGlobal RNG (seed 123): drawUniform = {val:.4f}")
