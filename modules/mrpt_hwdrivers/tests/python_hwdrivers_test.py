#!/usr/bin/env python3
"""Smoke tests for mrpt.hwdrivers Python bindings (no hardware needed)."""
import sys

try:
    from mrpt.hwdrivers import (
        CGenericSensor, CTaoboticsIMU, CGPSInterface, CVelodyneScanner, CJoystick,
    )
except ImportError as e:
    msg = str(e)
    if "_bindings" in msg and "No module named" in msg:
        print(f"SKIP: mrpt.hwdrivers bindings not built ({e})", file=sys.stderr)
        sys.exit(0)
    print(f"FAIL: mrpt.hwdrivers import error: {e}", file=sys.stderr)
    sys.exit(1)

from mrpt.config import CConfigFileMemory

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        print(f"  PASS  {name}"); PASS += 1
    else:
        print(f"  FAIL  {name}" + (f": {detail}" if detail else "")); FAIL += 1


print("Sensor factory")
for cls_name in ["CGPSInterface", "CVelodyneScanner", "CHokuyoURG", "CTaoboticsIMU"]:
    s_ = CGenericSensor.createSensor(cls_name)
    check(f"factory creates {cls_name}", s_ is not None and s_.getClassName() == cls_name)
gps = CGenericSensor.createSensor("CGPSInterface")
check("createSensor returns the concrete class", isinstance(gps, CGPSInterface), f"got {gps!r}")
check("createSensor unknown -> None", CGenericSensor.createSensor("NoSuchSensor") is None)

print("Configuration")
cfg = CConfigFileMemory(
    "[GPS]\nsensorLabel=MY_GPS\nCOM_port_LIN=/dev/ttyNONEXISTENT\nCOM_port_WIN=COM99\nbaudRate=4800\n")
gps.loadConfig(cfg, "GPS")
check("sensorLabel from config", gps.getSensorLabel() == "MY_GPS", f"got {gps.getSensorLabel()}")
check("serial port from config", gps.getSerialPortName() in ("/dev/ttyNONEXISTENT", "COM99"),
      f"got {gps.getSerialPortName()}")
check("class name", gps.getClassName() == "CGPSInterface")
check("no observations yet", gps.getObservations() == [])
check("initial state", gps.getState() in (CGenericSensor.ssInitializing, CGenericSensor.ssUninitialized))

print("Specific drivers")
imu = CTaoboticsIMU()
imu.setSerialPort("/dev/ttyNONEXISTENT")
imu.setSensorLabel("IMU")
check("CTaoboticsIMU", imu.getSensorLabel() == "IMU" and isinstance(imu, CGenericSensor))
velo = CVelodyneScanner()
velo.setModelName(CVelodyneScanner.VLP16)
check("CVelodyneScanner", velo.getClassName() == "CVelodyneScanner")
check("CJoystick count", CJoystick.getJoysticksCount() >= 0)

print(f"\nResults: {PASS} passed, {FAIL} failed")
sys.exit(1 if FAIL else 0)
