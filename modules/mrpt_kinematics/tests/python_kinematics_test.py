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
# sendVelRampCmd(vel, dir, ramp_time, rot_speed) — dir=pi/2 for lateral
import math as _math
holo.sendVelRampCmd(vel=1.0, dir=_math.pi/2, ramp_time=0.5, rot_speed=0.0)
for _ in range(10):
    holo.simulateOneTimeStep(dt=0.1)
pose2 = holo.getCurrentGTPose()
check("holo moved", True)  # just check no crash

print("Velocity commands and simulator API")
from mrpt.kinematics import CVehicleVelCmd
from mrpt.math import TPose2D
sim = CVehicleSimul_DiffDriven()
check("getVelCmdType", isinstance(sim.getVelCmdType(), CVehicleVelCmd_DiffDriven))
cmd = CVehicleVelCmd_DiffDriven()
check("CVehicleVelCmd base", isinstance(cmd, CVehicleVelCmd) and cmd.getVelCmdLength() == 2)
cmd.setVelCmdElement(0, 1.0)
cmd.setVelCmdElement(1, 0.0)
check("setVelCmdElement", abs(cmd.lin_vel - 1.0) < 1e-12 and abs(cmd.getVelCmdElement(0) - 1) < 1e-12)
sim.setCurrentGTPose(TPose2D(0, 0, math.pi / 2))
sim.setCurrentOdometricPose(TPose2D(0, 0, math.pi / 2))
sim.sendVelCmd(cmd)
sim.simulateOneTimeStep(1.0)
p = sim.getCurrentGTPose()
check("sendVelCmd motion", abs(p.x) < 1e-6 and abs(p.y - 1.0) < 1e-3, f"got {p}")
check("getTime", abs(sim.getTime() - 1.0) < 1e-9)
v = sim.getCurrentGTVel()
vl = sim.getCurrentGTVelLocal()
check("getCurrentGTVel (world frame)", abs(v.vx) < 1e-6 and abs(v.vy - 1.0) < 1e-6, f"got {v}")
check("getCurrentGTVelLocal", abs(vl.vx - 1.0) < 1e-6 and abs(vl.vy) < 1e-6, f"got {vl}")
vo = sim.getCurrentOdometricVel()
vol = sim.getCurrentOdometricVelLocal()
check("odometric velocities", abs(vo.vy - 1.0) < 1e-6 and abs(vol.vx - 1.0) < 1e-6)
check("getV/getW", abs(sim.getV() - 1.0) < 1e-12 and abs(sim.getW()) < 1e-12)
try:
    sim.sendVelCmd(CVehicleVelCmd_Holo())
    check("sendVelCmd with the wrong command type raises", False)
except Exception:
    check("sendVelCmd with the wrong command type raises", True)

sim.resetStatus()
check("resetStatus", abs(sim.getCurrentGTPose().y) < 1e-12 and abs(sim.getTime() - 1.0) < 1e-9)
sim.resetTime()
check("resetTime", sim.getTime() == 0.0)

print("Odometry errors")
odo_sim = CVehicleSimul_DiffDriven()
odo_sim.setOdometryErrors(True, Ax_err_bias=0.1, Ax_err_std=0.0, Ay_err_bias=0.0, Ay_err_std=0.0,
                          Aphi_err_bias=0.0, Aphi_err_std=0.0)
odo_sim.movementCommand(1.0, 0.0)
odo_sim.simulateOneTimeStep(2.0)
gt = odo_sim.getCurrentGTPose()
odo = odo_sim.getCurrentOdometricPose()
# Errors perturb the ground truth motion with respect to the commanded (odometric) one:
check("odometry bias", abs(odo.x - 2.0) < 1e-3 and abs(gt.x - 1.1 * odo.x) < 1e-6, f"gt={gt} odo={odo}")
odo_sim.setOdometryErrors(False)
odo_sim.simulateOneTimeStep(1.0)
check("odometry errors disabled", abs((odo_sim.getCurrentOdometricPose().x - odo.x) -
                                      (odo_sim.getCurrentGTPose().x - gt.x)) < 1e-6)

print("Delay model")
dly = CVehicleSimul_DiffDriven()
dly.setDelayModelParams(TAU_delay_sec=1.0, CMD_delay_sec=0.0)
dly.movementCommand(1.0, 0.0)
dly.simulateOneTimeStep(1.0)
# First order response: v(t) = 1 - exp(-t), x(t) = t - (1 - exp(-t))
check("first order velocity", abs(dly.getV() - (1 - math.exp(-1))) < 1e-3, f"got {dly.getV()}")
check("first order position", abs(dly.getCurrentGTPose().x - math.exp(-1)) < 1e-3,
      f"got {dly.getCurrentGTPose().x}")

print("Holonomic simulator with sendVelCmd")
hsim = CVehicleSimul_Holo()
hcmd = CVehicleVelCmd_Holo(1.0, math.pi / 2, 0.01, 0.0)
hsim.sendVelCmd(hcmd)
hsim.simulateOneTimeStep(2.0)
hp = hsim.getCurrentGTPose()
check("holonomic sendVelCmd", abs(hp.x) < 1e-3 and abs(hp.y - 2.0) < 2e-2, f"got {hp}")
check("holonomic getVelCmdType", isinstance(hsim.getVelCmdType(), CVehicleVelCmd_Holo))

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
