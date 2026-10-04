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
from __future__ import annotations
import mrpt as mrpt
from mrpt.hwdrivers._bindings import CGPSInterface as CGPSInterface
from mrpt.hwdrivers._bindings import CGenericSensor as CGenericSensor
from mrpt.hwdrivers._bindings import CHokuyoURG as CHokuyoURG
from mrpt.hwdrivers._bindings import CJoystick as CJoystick
from mrpt.hwdrivers._bindings import CRoboPeakLidar as CRoboPeakLidar
from mrpt.hwdrivers._bindings import CTaoboticsIMU as CTaoboticsIMU
from mrpt.hwdrivers._bindings import CVelodyneScanner as CVelodyneScanner
from . import _bindings
__all__: list = ['CGenericSensor', 'CTaoboticsIMU', 'CGPSInterface', 'CHokuyoURG', 'CRoboPeakLidar', 'CVelodyneScanner', 'CJoystick']
