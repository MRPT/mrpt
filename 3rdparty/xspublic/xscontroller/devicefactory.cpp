
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "devicefactory.h"

#include <xstypes/xsdeviceid.h>
#include "xsdevice_def.h"
#include "communicator.h"
#include "devicetypes.h"
#include "mtixdevice.h"
#include "mtix0device.h"
#include "mtix00device.h"
#include "mtigdevice.h"
#include "mti7device.h"
#include "mti6x0device.h"

/*! \class DeviceFactory
	\brief A Factory for the devices
*/

/*! \brief Default constructor
*/
DeviceFactory::DeviceFactory() : m_deviceManager(nullptr)
{
}

/*! \brief Registers a master device type.
	After registration, the factory is able to cerate an instance of the specified type using the deviceTypeId.
	A master device is not (yet) owned by an object
	\param[in] deviceTypeId the unique identifier for this device type
	\param[in] constructFunc the function used to create an instance of the required type
	\returns true when registration was successful
*/
bool DeviceFactory::registerMasterDeviceType(DeviceTypeId deviceTypeId, MasterConstructFunc constructFunc)
{
	bool rv = m_masterConstructors.insert(std::make_pair(deviceTypeId, constructFunc)).second;
	// it should be a master XOR a standalone (or neither)
	assert(m_standaloneConstructors.find(deviceTypeId) == m_standaloneConstructors.end());
	return rv;
}

/*! \brief Registers a standalone device type.
	After registration, the factory is able to cerate an instance of the specified type using the deviceTypeId.
	A master device is not (yet) owned by an object
	\param[in] deviceTypeId the unique identifier for this device type
	\param[in] constructFunc the function used to create an instance of the required type
	\returns true when registration was successful
*/
bool DeviceFactory::registerStandaloneDeviceType(DeviceTypeId deviceTypeId, StandaloneConstructFunc constructFunc)
{
	m_standaloneConstructors[deviceTypeId] = constructFunc;
	// it should be a master XOR a standalone (or neither)
	assert(m_masterConstructors.find(deviceTypeId) == m_masterConstructors.end());
	return true;
}

/*! \brief Constructs a master device of a specified deviceTypeId with a specified communicator.
	The created device takes ownership of the Communicator.
	\param[in] deviceTypeId the unique identifier for this device type
	\param[in] comm The communicator to used for this device
	\returns the newly created device or null when the device was not created.
*/
XsDevice* DeviceFactory::constructDevice(DeviceTypeId deviceTypeId, Communicator* comm)
{
	XsDevice* device = nullptr;
	if (deviceTypeId != DeviceType::INVALID)
	{
		auto i = m_masterConstructors.find(deviceTypeId);
		if (i != m_masterConstructors.end())
		{
			device = (i->second)(*this, comm);
			device->addRef();
		}
		else
		{
			auto j = m_standaloneConstructors.find(deviceTypeId);
			if (j != m_standaloneConstructors.end())
			{
				device = (j->second)(comm);
				device->addRef();
			}
		}
	}
	return device;
}

	/*! \brief Creates and initializes a master device with a specified communicator.
		The type of the new device is retrieved from the device id if the communicator.
		After construction the device will be initialized.
		\param[in] communicator The communicator, the function always takes ownership of this pointer
		\param[in] doInitialize If false, the device will not be initialized
		\returns the newly created device or null when the device was not created or could not be initialized
		\note the initializeDevice method should be overridden in derived classes.
		\note when a created device can not be initialized, it will be deleted again along with the supplied communicator
*/
XsDevice* DeviceFactory::createMasterDevice(Communicator* communicator, bool doInitialize)
{
	if (communicator != nullptr)
	{
		DeviceTypeId deviceTypeId = deviceToTypeId(communicator->masterDeviceId());
		XsDevice* constructedDevice = constructDevice(deviceTypeId, communicator);

		if (!doInitialize || initializeDevice(constructedDevice))
		{
#ifndef XSENS_DEBUG
			// this is support debug info, we don't want to spam the debugger windows
			JLWRITEG("Created master device with id: " << constructedDevice->deviceId() << " and firmware version: "<< constructedDevice->firmwareVersion().toString());
#endif
			return constructedDevice;
		}

		// construction failed
		if (constructedDevice)
			constructedDevice->removeRef();
		else
			communicator->destroy();
	}
	return nullptr;
}

/*! \brief converts an XsDeviceId to an DeviceTypeId
	\param[in] deviceId the deviceId
	\returns the DeviceTypeId
*/
DeviceFactory::DeviceTypeId DeviceFactory::deviceToTypeId(XsDeviceId const & deviceId) const
{
	if (deviceId.isMti() || deviceId.isMtig())
	{
		if (deviceId.isMtig()) return DeviceType::MTIG;
		if (deviceId.isMtiX00()) return DeviceType::MTI_X00;
		if (deviceId.isMtiX0()) return DeviceType::MTI_X0;
		if (deviceId.isMtiX() && deviceId.isGnss()) return DeviceType::MTI_7;
		if (deviceId.isMtiX()) return DeviceType::MTI_X;
		if (deviceId.isMti6X0()) return DeviceType::MTI_6X0;
	}

	return DeviceType::INVALID;
}

/*! \brief Tell our device manager to remove any devices matching \a deviceId
*/
void DeviceFactory::removeExistingDevice(const XsDeviceId &deviceId)
{
	if (m_deviceManager)
		m_deviceManager->removeExistingDevice(deviceId);
}

/*! \brief Initializes a device.
	Calls initialize(m_loadedScenarioFile) on the device
	\param[in] dev The device to initialize.
	\returns true if initialization is successful.
	*/
bool DeviceFactory::initializeDevice(XsDevice* dev) const
{
	if (dev && dev->initialize())
		return true;
	return false;
}

/*! \brief Initializes a device (if not already initialized)
	\param[in] device The device to initialize
	\returns true when the device is properly initialized or already initialized, false if initialization failed
*/
bool DeviceFactory::initializeDevice(XsDevice& device) const
{
	return device.isInitialized() ? true : initializeDevice(&device);
}

/*! \brief register all known device types
*/
void DeviceFactory::registerDevices()
{
	(void)registerStandaloneDeviceType(DeviceType::MTI_X,			&MtiXDevice::constructStandalone);
	(void)registerStandaloneDeviceType(DeviceType::MTI_X0,			&MtiX0Device::constructStandalone);
	(void)registerStandaloneDeviceType(DeviceType::MTI_X00,			&MtiX00Device::constructStandalone);
	(void)registerStandaloneDeviceType(DeviceType::MTIG,			&MtigDevice::constructStandalone);
	(void)registerStandaloneDeviceType(DeviceType::MTI_7,			&Mti7Device::constructStandalone);
	(void)registerStandaloneDeviceType(DeviceType::MTI_6X0,			&Mti6X0Device::constructStandalone);
}
