#!/usr/bin/env python3

# ---------------------------------------------------------------------
# Install python3-mrpt, or test with a local build with:
# export PYTHONPATH=$HOME/code/mrpt/build-Release/:$PYTHONPATH
# ---------------------------------------------------------------------

from mrpt import pymrpt as m

imu = m.mrpt.hwdrivers.CTaoboticsIMU()

imu.setSerialPort('/dev/ttyUSB0')

# Opens the serial port and start listening
imu.initialize()

while True:
    # Process serial port data queue:
    imu.doProcess()

    # Get observations:
    obsList = imu.getObservations()

    # Process them:
    if not obsList.empty():
        print('Read {} observations'.format(obsList.size()))
