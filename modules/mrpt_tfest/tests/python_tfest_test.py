#!/usr/bin/env python3
"""Smoke tests for mrpt.tfest Python bindings."""
import sys, math

try:
    from mrpt.tfest import TMatchingPair, TMatchingPairList, se2_l2
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.tfest bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.tfest import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("TMatchingPair / TMatchingPairList")
pair = TMatchingPair()
pair.localIdx = 0
pair.globalIdx = 0
pair.local_x = 0.0;  pair.local_y = 0.0
pair.global_x = 1.0; pair.global_y = 0.0
check("pair global_x", abs(pair.global_x - 1.0) < 1e-12)

lst = TMatchingPairList()
lst.push_back(pair)
check("list size == 1", len(lst) == 1)

print("se2_l2 — pure translation")
correspondences = TMatchingPairList()
# Two point pairs that differ by (2, 3) with no rotation
for lx, ly, gx, gy in [(0, 0, 2, 3), (1, 0, 3, 3), (0, 1, 2, 4)]:
    p = TMatchingPair()
    p.local_x = lx; p.local_y = ly
    p.global_x = gx; p.global_y = gy
    correspondences.push_back(p)

result = se2_l2(correspondences)
check("tx ≈ 2", abs(result.x - 2.0) < 0.01, f"tx={result.x:.4f}")
check("ty ≈ 3", abs(result.y - 3.0) < 0.01, f"ty={result.y:.4f}")
check("phi ≈ 0", abs(result.phi) < 0.01, f"phi={result.phi:.4f}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
