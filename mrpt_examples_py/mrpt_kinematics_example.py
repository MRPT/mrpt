#!/usr/bin/env python3
"""
mrpt_kinematics_example.py — vehicle kinematic simulators with mrpt.kinematics.

Demonstrates:
  - CVehicleVelCmd_DiffDriven: diff-drive velocity command
  - CVehicleVelCmd_Holo: holonomic velocity command
  - CVehicleSimul_DiffDriven: simulate a diff-drive robot over time
  - CVehicleSimul_Holo: simulate a holonomic robot
"""

import math
from mrpt.kinematics import (
    CVehicleVelCmd_DiffDriven,
    CVehicleVelCmd_Holo,
    CVehicleSimul_DiffDriven,
    CVehicleSimul_Holo,
)
from mrpt.math import TPose2D

# ---------------------------------------------------------------------------
# Velocity commands
# ---------------------------------------------------------------------------
cmd_diff = CVehicleVelCmd_DiffDriven()
cmd_diff.lin_vel = 1.0   # 1 m/s forward
cmd_diff.ang_vel = 0.2   # slight left turn
print(f"DiffDrive cmd: {cmd_diff}")
assert not cmd_diff.isStopCmd()

cmd_diff.setToStop()
assert cmd_diff.isStopCmd()
print("  setToStop ✓")

cmd_holo = CVehicleVelCmd_Holo(1.0, 0.0, 0.5, 0.1)  # vel, dir, ramp_time, rot_speed
print(f"\nHolo cmd: {cmd_holo}")
assert not cmd_holo.isStopCmd()

# ---------------------------------------------------------------------------
# Diff-drive simulator
# ---------------------------------------------------------------------------
sim_diff = CVehicleSimul_DiffDriven()
sim_diff.movementCommand(1.0, 0.0)   # straight ahead at 1 m/s

dt = 0.1
for _ in range(20):                  # 2 seconds
    sim_diff.simulateOneTimeStep(dt)

pose = sim_diff.getCurrentGTPose()
print(f"\nDiffDrive after 2 s at 1 m/s straight:")
print(f"  GT pose: x={pose.x:.2f} y={pose.y:.2f} phi={math.degrees(pose.phi):.1f} deg")
assert abs(pose.x - 2.0) < 0.05, f"Expected x≈2.0, got {pose.x}"
print("  position check ✓")

# ---------------------------------------------------------------------------
# Holonomic simulator
# ---------------------------------------------------------------------------
sim_holo = CVehicleSimul_Holo()
sim_holo.sendVelRampCmd(1.0, 0.0, 0.5, 0.0)   # vel=1, dir=0 (fwd), ramp=0.5s, no rotation

for _ in range(20):
    sim_holo.simulateOneTimeStep(dt)

pose_h = sim_holo.getCurrentGTPose()
print(f"\nHolo after 2 s:")
print(f"  GT pose: x={pose_h.x:.2f} y={pose_h.y:.2f} phi={math.degrees(pose_h.phi):.1f} deg")
print(f"  {sim_holo}")
