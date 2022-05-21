/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2022, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"	// Precompiled headers
//
#define MRPT_NO_WARN_BIG_HDR
#include <mrpt/core/initializer.h>
#include <mrpt/hwdrivers.h>
#include <mrpt/hwdrivers/registerAllClasses.h>
// deps:
#include <mrpt/comms/registerAllClasses.h>
#include <mrpt/gui/registerAllClasses.h>
#include <mrpt/maps/registerAllClasses.h>
#include <mrpt/vision/registerAllClasses.h>

MRPT_INITIALIZER(registerAllClasses_mrpt_hwdrivers)
{
	using namespace mrpt::hwdrivers;

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

void mrpt::hwdrivers::registerAllClasses_mrpt_hwdrivers()
{
	::registerAllClasses_mrpt_hwdrivers();
	// deps:
	mrpt::comms::registerAllClasses_mrpt_comms();
	mrpt::maps::registerAllClasses_mrpt_maps();
	mrpt::gui::registerAllClasses_mrpt_gui();
	mrpt::vision::registerAllClasses_mrpt_vision();
}
