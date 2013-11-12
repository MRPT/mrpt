/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
#include <mrpt/hwdrivers/CStereoGrabber_Bumblebee.h>
#include <mrpt/hwdrivers/CStereoGrabber_SVS.h>
#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/hwdrivers/CInterfaceNI845x.h>
#include <mrpt/hwdrivers/CKinect.h>

#include <mrpt/hwdrivers/CCANBusReader.h>

#endif
