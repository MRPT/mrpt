/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/initializer.h>
#include <mrpt/hwdrivers/CCANBusReader.h>
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/hwdrivers/CEnoseModular.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/hwdrivers/CGPS_NTRIP.h>
#include <mrpt/hwdrivers/CGillAnemometer.h>
#include <mrpt/hwdrivers/CGyroKVHDSP3000.h>
#include <mrpt/hwdrivers/CHokuyoURG.h>
#include <mrpt/hwdrivers/CIbeoLuxETH.h>
#include <mrpt/hwdrivers/CIMUXSens_MT4.h>
#include <mrpt/hwdrivers/CImpinjRFID.h>
#include <mrpt/hwdrivers/CKinect.h>
#include <mrpt/hwdrivers/CLMS100eth.h>
#include <mrpt/hwdrivers/CNTRIPEmitter.h>
#include <mrpt/hwdrivers/CNationalInstrumentsDAQ.h>
#include <mrpt/hwdrivers/COpenNI2Sensor.h>
#include <mrpt/hwdrivers/COpenNI2_RGBD360.h>
#include <mrpt/hwdrivers/CPhidgetInterfaceKitProximitySensors.h>
#include <mrpt/hwdrivers/CRaePID.h>
#include <mrpt/hwdrivers/CRoboPeakLidar.h>
#include <mrpt/hwdrivers/CSICKTim561Eth_2050101.h>
#include <mrpt/hwdrivers/CSickLaserSerial.h>
#include <mrpt/hwdrivers/CSickLaserUSB.h>
#include <mrpt/hwdrivers/CTaoboticsIMU.h>
#include <mrpt/hwdrivers/CVelodyneScanner.h>
#include <mrpt/hwdrivers/CWirelessPower.h>
#include <mrpt/hwdrivers/registerAllClasses.h>
#include <mrpt/serialization/CSerializable.h>
// deps:
#include <mrpt/comms/registerAllClasses.h>
#include <mrpt/maps/registerAllClasses.h>
#include <mrpt/viz/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_hwdrivers)
{
  using namespace mrpt::hwdrivers;

#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
  CSickLaserUSB::doRegister();
  CIbeoLuxETH::doRegister();
  CHokuyoURG::doRegister();
  CRoboPeakLidar::doRegister();
  CGPSInterface::doRegister();
  CIMUXSens_MT4::doRegister();
  CCameraSensor::doRegister();
  CWirelessPower::doRegister();
  CRaePID::doRegister();
  CImpinjRFID::doRegister();
  CSickLaserSerial::doRegister();
  CEnoseModular::doRegister();
  CGillAnemometer::doRegister();
  CNTRIPEmitter::doRegister();
  CLMS100Eth::doRegister();
  CPhidgetInterfaceKitProximitySensors::doRegister();
  CGyroKVHDSP3000::doRegister();
  CKinect::doRegister();
  COpenNI2Sensor::doRegister();
  COpenNI2_RGBD360::doRegister();
  CCANBusReader::doRegister();
  CNationalInstrumentsDAQ::doRegister();
  CGPS_NTRIP::doRegister();
  CVelodyneScanner::doRegister();
  CSICKTim561Eth::doRegister();
  CTaoboticsIMU::doRegister();
#endif
}

void mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers()
{
  ::registerAllClasses_mrpt_hwdrivers();
  // deps:
  mrpt::comms::registerAllClasses_mrpt_comms();
  mrpt::maps::registerAllClasses_mrpt_maps();
  mrpt::viz::registerAllClasses_mrpt_viz();
}
