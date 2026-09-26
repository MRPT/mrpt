#!/usr/bin/env python3

# Example: read IMU data from a Taobotics USB IMU sensor.
#
# Usage:
#   . install/setup.bash
#   ./mrpt_examples_py/hwdriver-tao-imu-usb.py [/dev/ttyUSB0]
#
# Any other sensor driver can be used the same way, configured from an .ini
# file section instead of with setters:
#   sensor = CGenericSensor.createSensor("CGPSInterface")
#   sensor.loadConfig(mrpt.config.CConfigFile("sensors.ini"), "GPS")

import sys
import time

from mrpt.core import Clock
from mrpt.hwdrivers import CGenericSensor, CTaoboticsIMU
from mrpt.obs import CObservationIMU, TIMUDataIndex

serial_port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

imu = CTaoboticsIMU()
imu.setSerialPort(serial_port)
imu.initialize()
if imu.getState() == CGenericSensor.ssError:
    sys.exit(f"Could not open the IMU at {serial_port}")

while True:
    imu.doProcess()
    for timestamp, obs in imu.getObservations():
        if not isinstance(obs, CObservationIMU):
            continue
        print(f"t={Clock.toDouble(timestamp):.3f} "
              f"wz={obs.get(TIMUDataIndex.IMU_WZ):+.4f} rad/s "
              f"acc_z={obs.get(TIMUDataIndex.IMU_Z_ACC):+.3f} m/s2")
    time.sleep(0.005)  # poll often: data is buffered by the driver between calls
