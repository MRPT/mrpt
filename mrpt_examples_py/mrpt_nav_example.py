#!/usr/bin/env python3
"""
mrpt_nav_example.py -- Navigation primitives with mrpt.nav.

Demonstrates:
  - TWaypoint / TWaypointSequence / TWaypointStatus
  - CParameterizedTrajectoryGenerator factory
  - CLogFileRecord
"""

import mrpt.nav as nav
from mrpt.math import TPoint2D

# ---------------------------------------------------------------------------
# TWaypoint / TWaypointSequence
# ---------------------------------------------------------------------------
wp1 = nav.TWaypoint()
wp1.target = TPoint2D(5.0, 3.0)
wp1.allowed_distance = 0.5
wp1.allow_skip = True
print(f"Waypoint 1: target=({wp1.target.x}, {wp1.target.y}), tol={wp1.allowed_distance}")

wp2 = nav.TWaypoint()
wp2.target = TPoint2D(10.0, 0.0)
wp2.allowed_distance = 0.3

seq = nav.TWaypointSequence()
seq.append(wp1)
seq.append(wp2)
print(f"Waypoint sequence ({len(seq.waypoints)} waypoints):")
print(seq.getAsText())

# ---------------------------------------------------------------------------
# CParameterizedTrajectoryGenerator factory
# ---------------------------------------------------------------------------
# Build a minimal PTG config in INI format
ptg_ini = (
    "[PTG]\n"
    "K=1.0\n"
    "v_max_mps=0.5\n"
    "w_max_dps=60\n"
    "T_c_deg_s=0.5\n"
    "turningRadiusReference=0.10\n"
    "num_paths=31\n"
    "refDistance=3.0\n"
    "resolution=0.10\n"
    "score_priority=1.0\n"
)
ptg = nav.CParameterizedTrajectoryGenerator.CreatePTG("CPTG_DiffDrive_C", ptg_ini, "PTG")
if ptg:
    # initialize() requires a robot shape definition; skip here for brevity
    desc = ptg.getDescription()
    print(f"PTG created: {type(ptg).__name__}")
    print(f"  description: {desc}")
    print(f"  isInitialized: {ptg.isInitialized()}")
else:
    print("PTG creation returned None")

# ---------------------------------------------------------------------------
# CLogFileRecord
# ---------------------------------------------------------------------------
log = nav.CLogFileRecord()
print(f"CLogFileRecord created (type={type(log).__name__})")

print("\nAll mrpt.nav examples passed.")
