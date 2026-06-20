#!/usr/bin/env python3
"""Smoke tests for mrpt.slam Python bindings."""
import sys, math

try:
    from mrpt.slam import CICP, CMetricMapBuilderICP, TICPAlgorithm
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.slam bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.slam import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CICP")
icp = CICP()
check("CICP created", icp is not None)
check("default algo exists", hasattr(icp.options, "ICP_algorithm"))
icp.options.ICP_algorithm = TICPAlgorithm.icpClassic
check("algo setter", icp.options.ICP_algorithm == TICPAlgorithm.icpClassic)

print("CMetricMapBuilderICP")
builder = CMetricMapBuilderICP()
builder.ICP_options.insertionLinDistance = 0.5
builder.ICP_options.insertionAngDistance = math.radians(30)
builder.initialize()
check("initialize ok", True)

pose_pdf = builder.getCurrentPoseEstimation()
check("getCurrentPoseEstimation not None", pose_pdf is not None)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
