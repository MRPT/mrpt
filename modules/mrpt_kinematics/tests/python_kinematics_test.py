#!/usr/bin/env python3
"""Smoke tests for mrpt.kinematics Python bindings."""
import sys, math

try:
    from mrpt.kinematics import (
        CVehicleSimul_DiffDriven, CVehicleSimul_Holo,
        CVehicleVelCmd_DiffDriven, CVehicleVelCmd_Holo,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.kinematics bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.kinematics import error: {e}", file=sys.stderr)
    sys.exit(1)

PASS = FAIL = 0

def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1

print("CVehicleSimul_DiffDriven")
robot = CVehicleSimul_DiffDriven()
robot.movementCommand(lin_vel=1.0, ang_vel=0.0)
for _ in range(10):
    robot.simulateOneTimeStep(dt=0.1)
pose = robot.getCurrentGTPose()
check("x > 0 after forward motion", pose.x > 0.5, f"x={pose.x:.3f}")
check("phi ~ 0", abs(pose.phi) < 0.01, f"phi={pose.phi:.4f}")

print("CVehicleSimul_Holo")
holo = CVehicleSimul_Holo()
holo.movementCommand(vel_x=0.0, vel_y=1.0, omega=0.0)
for _ in range(10):
    holo.simulateOneTimeStep(dt=0.1)
pose2 = holo.getCurrentGTPose()
check("y > 0 after lateral motion", pose2.y > 0.5, f"y={pose2.y:.3f}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
