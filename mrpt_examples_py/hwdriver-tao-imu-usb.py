#!/usr/bin/env python3

# Example: read IMU data from a Taobotics USB IMU sensor.
#
# NOTE: mrpt.hwdrivers bindings not yet implemented (pybind11_plan_v3.md §2.7).
# TODO: wrap mrpt.hwdrivers.CTaoboticsIMU, CGenericSensor.

# from mrpt.hwdrivers import CTaoboticsIMU
# from mrpt.core import Clock
#
# imu = CTaoboticsIMU()
# imu.setSerialPort('/dev/ttyUSB0')
# imu.initialize()
#
# while True:
#     imu.doProcess()
#     obsList = imu.getObservations()
#     if not obsList.empty():
#         print('Read {} observations'.format(obsList.size()))
#         for t, obs in obsList:
#             timestamp = Clock.toDouble(t)
#             print('Time={} obs={}'.format(timestamp, obs.asString()))
#             # Access individual IMU readings:
#             # from mrpt.obs import CObservationIMU, TIMUDataIndex
#             # o = CObservationIMU(obs)
#             # angVel_z = o.get(TIMUDataIndex.IMU_WZ)
