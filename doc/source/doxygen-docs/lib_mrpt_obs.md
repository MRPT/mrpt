\defgroup mrpt_obs_grp [mrpt-obs]

Observation classes for all kinds of robot sensors.

[TOC]

# Library mrpt-obs

This library is part of MRPT and can be installed in Debian-based systems with:

		sudo apt install libmrpt-obs-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

Key concepts:

- **Sensor observations**: All sensor observations share a common virtual
  base class (mrpt::obs::CObservation). There are classes to store 2D/3D laser
  scans, rotating LiDAR scans, monocular and stereo images, depth cameras, GPS,
  IMU, odometry, gas sensors, WiFi/RFID, and more.
  A concept closely related to observations is mrpt::obs::CSensoryFrame — a
  set of observations collected approximately at the same instant.

- **Rawlogs (datasets)**: A robotics dataset can be loaded, edited and
  explored by means of mrpt::obs::CRawlog. See also:
  https://www.mrpt.org/Rawlog_Format

- **Actions**: For Bayesian filtering, robot actions (e.g. 2D odometry
  increments) can be represented as mrpt::obs::CAction objects.

- **Simple maps**: A `CSimpleMap` is a sequence of `(pose PDF, sensory frame)`
  pairs. Metric maps can be rebuilt on-demand from this compact representation.

- **CARMEN log tools**: Utilities to convert CARMEN log files to MRPT rawlogs.
  See mrpt::obs::carmen_log_parse_line and the `carmen2rawlog` / `carmen2simplemap`
  applications.

## Key observation classes (MRPT 3 API)

| Class | Sensor |
|-------|--------|
| mrpt::obs::CObservation2DRangeScan | 2D laser scanner (single scan arc) |
| mrpt::obs::CObservationRotatingScan | Rotating LiDAR (organized range/intensity images); replaces `CObservationVelodyneScan` for processing |
| mrpt::obs::CObservationVelodyneScan | Raw Velodyne packets; convert via `CObservationRotatingScan::fromVelodyne()` |
| mrpt::obs::CObservationPointCloud | Unstructured 3D point cloud |
| mrpt::obs::CObservationImage | Monocular camera image |
| mrpt::obs::CObservationStereoImages | Stereo pair |
| mrpt::obs::CObservation3DRangeScan | RGB-D (structured depth + color) |
| mrpt::obs::CObservationGPS | GPS/GNSS fix |
| mrpt::obs::CObservationIMU | IMU (accelerometer, gyro, magnetometer) |
| mrpt::obs::CObservationOdometry | Wheel odometry |
| mrpt::obs::CObservationGasSensors | Electronic nose / gas sensors |
| mrpt::obs::CObservationWirelessPower | WiFi RSSI |

# Library contents
