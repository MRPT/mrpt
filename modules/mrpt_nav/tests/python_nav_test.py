#!/usr/bin/env python3
"""Unit tests for the mrpt.nav Python bindings."""
import math
import os
import sys
import tempfile

try:
    from mrpt.nav import CParameterizedTrajectoryGenerator, TWaypoint, TWaypointSequence
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.nav bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.nav import error: {e}", file=sys.stderr)
    sys.exit(1)

from mrpt.kinematics import CVehicleVelCmd_DiffDriven, CVehicleVelCmd_Holo

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}")
        PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else ""))
        FAIL += 1


print("TWaypointSequence")
seq = TWaypointSequence()
seq.append(TWaypoint(1.0, 2.0, 0.5))
check("waypoints", len(seq) == 1 and abs(seq[0].target.y - 2.0) < 1e-12)

INI = """
[PTG]
holo_Type = CPTG_Holo_Blend
holo_refDistance = 5.0
holo_num_paths = 31
holo_T_ramp_max = 0.8
holo_v_max_mps = 1.0
holo_w_max_dps = 60
holo_robot_radius = 0.3

diff_Type = CPTG_DiffDrive_C
diff_resolution = 0.05
diff_refDistance = 3.0
diff_num_paths = 31
diff_v_max_mps = 1.0
diff_w_max_dps = 60
diff_K = 1.0
diff_shape_x0 = -0.2
diff_shape_x1 = 0.2
diff_shape_x2 = 0.2
diff_shape_x3 = -0.2
diff_shape_y0 = 0.2
diff_shape_y1 = 0.2
diff_shape_y2 = -0.2
diff_shape_y3 = -0.2
"""

print("CPTG_Holo_Blend")
holo = CParameterizedTrajectoryGenerator.CreatePTG("CPTG_Holo_Blend", INI, "PTG", "holo_")
holo.initialize()
N = holo.getPathCount()
k0 = N // 2  # alpha = 0: straight forward
check("initialize with defaults", holo.isInitialized() and N == 31)
check("index2alpha", abs(holo.index2alpha(k0)) < 1e-12 and
      abs(holo.index2alpha(0) - math.pi * (-1 + 1 / N)) < 1e-12)
check("alpha2index", all(holo.alpha2index(holo.index2alpha(k)) == k for k in range(N)))
check("getRefDistance", abs(holo.getRefDistance() - 5.0) < 1e-12)
check("max velocities", abs(holo.getMaxLinVel() - 1.0) < 1e-12 and
      abs(holo.getMaxAngVel() - math.radians(60)) < 1e-9)
n_steps = holo.getPathStepCount(k0)
check("getPathStepCount", n_steps > 10)
step = n_steps // 2
p = holo.getPathPose(k0, step)
d = holo.getPathDist(k0, step)
check("straight path pose", abs(p.y) < 1e-9 and abs(p.x - d) < 1e-6 and d > 0, f"{p} {d}")
check("path end at the reference distance",
      abs(holo.getPathDist(k0, n_steps - 1) - holo.getRefDistance()) < 0.05)
k1 = holo.alpha2index(math.pi / 2)
p1 = holo.getPathPose(k1, holo.getPathStepCount(k1) - 1)
alpha1 = holo.index2alpha(k1)
check("lateral path direction", abs(math.atan2(p1.y, p1.x) - alpha1) < 1e-6, f"{p1}")
tw = holo.getPathTwist(k0, n_steps - 1)
check("getPathTwist at the end", abs(tw.vx - 1.0) < 1e-3 and abs(tw.vy) < 1e-6, f"{tw}")
s_for = holo.getPathStepForDist(k0, 2.0)
# The last step whose distance is below the given one:
check("getPathStepForDist", s_for is not None and
      holo.getPathDist(k0, s_for) < 2.0 <= holo.getPathDist(k0, s_for + 1))
check("getPathStepDuration", holo.getPathStepDuration() > 0)
cmd = holo.directionToMotionCommand(k1)
check("directionToMotionCommand (holonomic)", isinstance(cmd, CVehicleVelCmd_Holo) and
      abs(cmd.dir_local - alpha1) < 1e-9 and abs(cmd.vel - 1.0) < 1e-9, f"{cmd}")

tp = holo.initTPObstacles()
check("initTPObstacles", len(tp) == N and all(abs(v - 5.0) < 1e-9 for v in tp), f"{tp[:3]}")
tp2 = holo.updateTPObstacle(2.0, 0.0, tp)
check("updateTPObstacle in front", abs(tp2[k0] - (2.0 - 0.3)) < 1e-3, f"got {tp2[k0]}")
check("updateTPObstacle does not block the opposite direction", abs(tp2[0] - 5.0) < 1e-9)
check("updateTPObstacle returns a copy", abs(tp[k0] - 5.0) < 1e-9)
tps = holo.updateTPObstacleSingle(2.0, 0.0, k0, holo.initTPObstacleSingle(k0))
check("updateTPObstacleSingle", abs(tps - tp2[k0]) < 1e-9)
try:
    holo.updateTPObstacle(2.0, 0.0, [5.0] * 3)
    check("updateTPObstacle with a wrong length", False, "no exception")
except ValueError:
    check("updateTPObstacle with a wrong length", True)
try:
    holo.directionToMotionCommand(N)
    check("directionToMotionCommand out of range", False, "no exception")
except IndexError:
    check("directionToMotionCommand out of range", True)
holo.deinitialize()
holo.setRefDistance(4.0)
holo.initialize()
check("setRefDistance", abs(holo.getRefDistance() - 4.0) < 1e-12 and
      abs(holo.initTPObstacles()[0] - 4.0) < 1e-9)

print("CPTG_DiffDrive_C with a cache file")
diff = CParameterizedTrajectoryGenerator.CreatePTG("CPTG_DiffDrive_C", INI, "PTG", "diff_")
with tempfile.TemporaryDirectory() as tmp:
    cache = os.path.join(tmp, "ptg_cache.bin.gz")
    diff.initialize(cacheFilename=cache)
    check("cache file written to the given path", os.path.isfile(cache) and diff.isInitialized())
    diff2 = CParameterizedTrajectoryGenerator.CreatePTG("CPTG_DiffDrive_C", INI, "PTG", "diff_")
    diff2.initialize(cache, False)
    check("PTG initialized from the cache", diff2.isInitialized() and
          diff2.getPathStepCount(15) == diff.getPathStepCount(15))
k0 = diff.getPathCount() // 2
last = diff.getPathStepCount(k0) - 1
p = diff.getPathPose(k0, last)
check("straight diff-drive path", abs(p.y) < 1e-6 and abs(p.x - diff.getPathDist(k0, last)) < 1e-3)
# Circular arcs of path k: the curvature w/v grows with |alpha|.
ka = diff.alpha2index(math.pi / 4)
pa = diff.getPathPose(ka, diff.getPathStepCount(ka) // 4)
tw = diff.getPathTwist(ka, 5)
radius = abs(math.hypot(tw.vx, tw.vy) / tw.omega)
check("arc of constant curvature", abs(math.hypot(pa.x, pa.y - radius) - radius) < 0.01,
      f"pose {pa}, radius {radius}")
s_for = diff.getPathStepForDist(k0, 1.5)
check("getPathStepForDist (diff-drive)", s_for is not None and
      diff.getPathDist(k0, s_for) < 1.5 <= diff.getPathDist(k0, s_for + 1))
check("getPathStepForDist beyond the path", diff.getPathStepForDist(k0, 100.0) is None)
cmd = diff.directionToMotionCommand(k0)
check("directionToMotionCommand (diff-drive)", isinstance(cmd, CVehicleVelCmd_DiffDriven) and
      abs(cmd.lin_vel - 1.0) < 1e-9 and abs(cmd.ang_vel) < 1e-9, f"{cmd}")
tp = diff.updateTPObstacle(1.0, 0.0, diff.initTPObstacles())
check("diff-drive TP-Obstacle in front", abs(tp[k0] - (1.0 - 0.2)) < 0.06, f"got {tp[k0]}")

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
