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

#include <mrpt/hwdrivers.h> // Precompiled headers

using namespace mrpt::utils;
using namespace mrpt::hwdrivers;


void registerAllClasses_mrpt_hwdrivers();

CStartUpClassesRegister  mrpt_hwdrivers_class_reg(&registerAllClasses_mrpt_hwdrivers);


/** Register existing sensors.
  * \sa mrpt::hwdrivers::CGenericSensor::createSensor
  */
void registerAllClasses_mrpt_hwdrivers()
{
	CSickLaserUSB::doRegister();
	CIbeoLuxETH::doRegister();
	CHokuyoURG::doRegister();
	CGPSInterface::doRegister();
	CBoardSonars::doRegister();
	CBoardIR::doRegister();
	CBoardDLMS::doRegister();
	CIMUXSens::doRegister();
	CCameraSensor::doRegister();
	CActivMediaRobotBase::doRegister();
	CPtuHokuyo::doRegister();
	CWirelessPower::doRegister();
	CSickLaserSerial::doRegister();
	CBoardENoses::doRegister();
	CNTRIPEmitter::doRegister();
	CSwissRanger3DCamera::doRegister();
	CLMS100Eth::doRegister();
	CPhidgetInterfaceKitProximitySensors::doRegister();
	CGyroKVHDSP3000::doRegister();
	CKinect::doRegister();
}

