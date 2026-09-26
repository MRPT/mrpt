#!/usr/bin/env python3
"""
mrpt_hwdrivers_example.py: sensor drivers from Python.

Any MRPT sensor driver can be created by class name and configured from an
.ini section, like the rawlog-grabber application does. This example only
configures a driver (no hardware needed); see hwdriver-tao-imu-usb.py for a
capture loop.

Run:
    python3 mrpt_hwdrivers_example.py
"""

from mrpt.config import CConfigFileMemory
from mrpt.hwdrivers import CGenericSensor, CJoystick

# ---------------------------------------------------------------------------
# Create and configure a driver by its class name
# ---------------------------------------------------------------------------
gps = CGenericSensor.createSensor("CGPSInterface")
gps.loadConfig(CConfigFileMemory("""
[GPS]
sensorLabel  = GPS_RTK
COM_port_LIN = /dev/ttyUSB0
COM_port_WIN = COM3
baudRate     = 115200
"""), "GPS")
print(f"{gps!r}: port={gps.getSerialPortName()}, rate={gps.getProcessRate()} Hz")

# With a device connected, the capture loop would be:
#   gps.initialize()
#   while True:
#       gps.doProcess()
#       for timestamp, obs in gps.getObservations():
#           print(obs)

# ---------------------------------------------------------------------------
# Joysticks / gamepads
# ---------------------------------------------------------------------------
n_joy = CJoystick.getJoysticksCount()
print(f"\n{n_joy} joystick(s) connected")
if n_joy > 0:
    state = CJoystick().getJoystickPosition(0)
    if state is not None:
        print(f"  axes={state.axes} buttons={state.buttons}")
