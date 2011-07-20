/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
#include <mrpt/hwdrivers/CSerialPort.h>
#include <mrpt/hwdrivers/CBoardDLMS.h>
#include <mrpt/hwdrivers/CBoardIR.h>
#include <mrpt/hwdrivers/CIMUXSens.h>
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
#include <mrpt/hwdrivers/CServoeNeck.h>
#include <mrpt/hwdrivers/CNTRIPEmitter.h>
#include <mrpt/hwdrivers/CRoboticHeadInterface.h>
#include <mrpt/hwdrivers/CRovio.h>
#include <mrpt/hwdrivers/CSwissRanger3DCamera.h>
#include <mrpt/hwdrivers/CGyroKVHDSP3000.h>

#include <mrpt/hwdrivers/CImageGrabber_dc1394.h>
#include <mrpt/hwdrivers/CImageGrabber_OpenCV.h>
#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee.h>
#include <mrpt/hwdrivers/CStereoGrabber_SVS.h>
#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/hwdrivers/CKinect.h>

#endif
