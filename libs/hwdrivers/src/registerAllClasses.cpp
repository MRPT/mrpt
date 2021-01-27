/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"	// Precompiled headers
//
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/core/initializer.h>
#include <mrpt/hwdrivers.h>

using namespace mrpt::hwdrivers;

MRPT_INITIALIZER(registerAllClasses_mrpt_hwdrivers)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	CSickLaserUSB::doRegister();
	CIbeoLuxETH::doRegister();
	CHokuyoURG::doRegister();
	CRoboPeakLidar::doRegister();
	CGPSInterface::doRegister();
	CBoardSonars::doRegister();
	CIMUXSens_MT4::doRegister();
	CCameraSensor::doRegister();
	CWirelessPower::doRegister();
	CRaePID::doRegister();
	CImpinjRFID::doRegister();
	CSickLaserSerial::doRegister();
	CBoardENoses::doRegister();
	CEnoseModular::doRegister();
	CGillAnemometer::doRegister();
	CNTRIPEmitter::doRegister();
	CSwissRanger3DCamera::doRegister();
	CLMS100Eth::doRegister();
	CPhidgetInterfaceKitProximitySensors::doRegister();
	CGyroKVHDSP3000::doRegister();
	CKinect::doRegister();
	COpenNI2Sensor::doRegister();
	COpenNI2_RGBD360::doRegister();
	CCANBusReader::doRegister();
	CNationalInstrumentsDAQ::doRegister();
	CGPS_NTRIP::doRegister();
	CIMUIntersense::doRegister();
	CSkeletonTracker::doRegister();
	CVelodyneScanner::doRegister();
	CSICKTim561Eth::doRegister();
#endif
}
