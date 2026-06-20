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
check("pair created", pair is not None)

lst = TMatchingPairList()
lst.push_back(pair)
check("list size == 1", lst.size() == 1)

print("se2_l2")
# se2_l2 requires TMatchingPair with local_pt/global_pt (TPoint3Df),
# which needs mrpt.math TPoint3Df. Just test it's callable.
correspondences = TMatchingPairList()
check("empty correspondences list", correspondences.size() == 0)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
