"""
mrpt.hwdrivers: sensor drivers.

Any driver can be created by class name and configured from an .ini section,
the same way `rawlog-grabber` does it::

    import mrpt.config, mrpt.hwdrivers as hw
    sensor = hw.CGenericSensor.createSensor("CGPSInterface")
    sensor.loadConfig(mrpt.config.CConfigFile("sensors.ini"), "GPS")
    sensor.initialize()
    while True:
        sensor.doProcess()
        for timestamp, obs in sensor.getObservations():
            print(obs)

A few drivers also have setters to be configured without a config file:
CTaoboticsIMU, CGPSInterface, CHokuyoURG, CRoboPeakLidar, CVelodyneScanner.
CJoystick reads joysticks and gamepads.
"""

import mrpt.config  # noqa: F401  (loadConfig() takes a CConfigFileBase)
import mrpt.obs     # noqa: F401  (drivers return mrpt.obs observations)

from mrpt.hwdrivers._bindings import (
    CGenericSensor,
    CTaoboticsIMU,
    CGPSInterface,
    CHokuyoURG,
    CRoboPeakLidar,
    CVelodyneScanner,
    CJoystick,
)

__all__ = [
    "CGenericSensor",
    "CTaoboticsIMU",
    "CGPSInterface",
    "CHokuyoURG",
    "CRoboPeakLidar",
    "CVelodyneScanner",
    "CJoystick",
]
