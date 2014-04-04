/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2014, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/**  This is the main "include file" for classes into the mrpt::hwdrivers namespace. This file
 *	   includes all the other ones, so user applications must include just this one
 *     and link against the library file "lib_hwdrivers.lib" / "lib_hwdrivers.a"
 */
#ifndef HWDRIVERS_H
#define HWDRIVERS_H

// Classes into HWDRIVERS
// --------------------------------------------
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/hwdrivers/C2DRangeFinderAbstract.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CSickLaserUSB.h>
#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include <mrpt/hwdrivers/CIbeoLuxETH.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/hwdrivers/CInterfaceFTDIMessages.h>
#include <mrpt/hwdrivers/CWirelessPower.h>
#include <mrpt/hwdrivers/CRaePID.h>
#include <mrpt/hwdrivers/CImpinjRFID.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/hwdrivers/CBoardDLMS.h>
#include <mrpt/hwdrivers/CBoardIR.h>
#include <mrpt/hwdrivers/CIMUXSens.h>
#include <mrpt/hwdrivers/CIMUXSens_MT4.h>
#include <mrpt/hwdrivers/CActivMediaRobotBase.h>
#include <mrpt/hwdrivers/CJoystick.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/hwdrivers/CPtuDPerception.h>
#include <mrpt/hwdrivers/CPtuHokuyo.h>
#include <mrpt/hwdrivers/CTuMicos.h>
#include <mrpt/hwdrivers/CFFMPEG_InputStream.h>
#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/hwdrivers/CLMS100eth.h>
#include <mrpt/hwdrivers/CBoardSonars.h>
#include <mrpt/hwdrivers/CBoardENoses.h>
#include <mrpt/hwdrivers/CEnoseModular.h>
#include <mrpt/hwdrivers/CServoeNeck.h>
#include <mrpt/hwdrivers/CNTRIPEmitter.h>
#include <mrpt/hwdrivers/CRoboticHeadInterface.h>
#include <mrpt/hwdrivers/CRovio.h>
#include <mrpt/hwdrivers/CSwissRanger3DCamera.h>
#include <mrpt/hwdrivers/CGyroKVHDSP3000.h>

#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#include <mrpt/hwdrivers/CImageGrabber_FlyCapture2.h>
#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee.h>
#include <mrpt/hwdrivers/CStereoGrabber_SVS.h>
#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/hwdrivers/COpenNI2_RGBD360.h>

#include <mrpt/hwdrivers/CCANBusReader.h>
#include <mrpt/hwdrivers/CNationalInstrumentsDAQ.h>

#endif
