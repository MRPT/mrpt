\defgroup mrpt_hwdrivers_grp [mrpt-hwdrivers]

Sensor and hardware-related drivers.

[TOC]

# Library mrpt-hwdrivers

This C++ library is part of MRPT and can be installed in Debian-based systems
with:

		sudo apt install libmrpt-hwdrivers-dev

Read also [how to import MRPT into your CMake scripts](mrpt_from_cmake.html).

This library provides sensor drivers and hardware interface utilities, from
low-level serial/USB helpers to high-level sensor wrappers.

All sensor drivers derive from mrpt::hwdrivers::CGenericSensor, which provides
a unified `doProcess()` / `getObservations()` interface and integrable into
`rawlog-grabber`.

## Supported sensors

| Class | Sensor / Protocol |
|-------|-------------------|
| mrpt::hwdrivers::CCameraSensor | Generic camera (OpenCV, IEEE1394, etc.) |
| mrpt::hwdrivers::CImageGrabber_OpenCV | OpenCV camera capture |
| mrpt::hwdrivers::CKinect | Microsoft Kinect (libfreenect) |
| mrpt::hwdrivers::COpenNI2Sensor | OpenNI2-compatible depth cameras |
| mrpt::hwdrivers::CMyntEyeCamera | MYNT EYE stereo/depth camera |
| mrpt::hwdrivers::CVelodyneScanner | Velodyne HDL-32E / VLP-16 LiDAR |
| mrpt::hwdrivers::CSickLaserUSB | SICK LMS/TIM lasers (USB) |
| mrpt::hwdrivers::CSickLaserSerial | SICK LMS lasers (serial) |
| mrpt::hwdrivers::CIbeoLuxETH | Ibeo Lux 4-layer LiDAR (Ethernet) |
| mrpt::hwdrivers::CGPSInterface | GPS/GNSS via NMEA or NTRIP |
| mrpt::hwdrivers::CGPS_NTRIP | Combined GPS + NTRIP correction stream |
| mrpt::hwdrivers::CIMUXSens_MT4 | XSens MTi IMU (MT4 protocol) |
| mrpt::hwdrivers::CGyroKVHDSP3000 | KVH DSP-3000 fiber-optic gyro |
| mrpt::hwdrivers::CTaoboticsIMU | Taobotics IMU |
| mrpt::hwdrivers::CGillAnemometer | Gill ultrasonic anemometer |
| mrpt::hwdrivers::CEnoseModular | Modular electronic nose |
| mrpt::hwdrivers::CRaePID | RAE PID gas detector |
| mrpt::hwdrivers::CWirelessPower | WiFi RSSI scanner |
| mrpt::hwdrivers::CPhidgetInterfaceKitProximitySensors | Phidget proximity sensors |
| mrpt::hwdrivers::CNationalInstrumentsDAQ | National Instruments DAQ |
| mrpt::hwdrivers::CCANBusReader | CAN bus raw reader |
| mrpt::hwdrivers::CNTRIPEmitter | NTRIP caster client |
| mrpt::hwdrivers::CImpinjRFID | Impinj RFID reader |

# Library contents
