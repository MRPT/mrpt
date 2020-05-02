
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

#ifndef DEVICE_FACTORY_H
#define DEVICE_FACTORY_H

#include <memory>
#include <map>
#include <xstypes/xsdeviceid.h>

struct Communicator;
struct XsDevice;

class DeviceFactory
{
public:

	/*! \class DeviceManager
		\brief An abstract class that is used for removing the existing devices
	*/
	class DeviceManager
	{
	public:
		/*! \brief Removes existing device
			\param deviceId The ID of device to remove
		*/
		virtual void removeExistingDevice(const XsDeviceId &deviceId) = 0;
	};

	DeviceFactory();
	virtual ~DeviceFactory() {}

	//! \brief Provides the device type ID as unsigned int
	typedef unsigned int DeviceTypeId;

	//! \brief A function prototype that provides the device factory and communicator
	typedef XsDevice* (*MasterConstructFunc)(DeviceFactory & deviceFactory, Communicator* comm);

	//! \brief A function prototype that provides the communicator
	typedef XsDevice* (*StandaloneConstructFunc)(Communicator* comm);

	virtual XsDevice* createMasterDevice(Communicator* communicator, bool doInitialize = true);

	virtual DeviceTypeId deviceToTypeId(XsDeviceId const & deviceId) const;

	bool registerStandaloneDeviceType(DeviceTypeId deviceTypeId, StandaloneConstructFunc constructFunc);
	bool registerMasterDeviceType(DeviceTypeId deviceTypeId, MasterConstructFunc constructFunc);

	bool initializeDevice(XsDevice& device) const;

	virtual void removeExistingDevice(XsDeviceId const & deviceId);
	virtual void registerDevices();

protected:
	virtual XsDevice* constructDevice(DeviceTypeId deviceTypeId, Communicator* comm);

	virtual bool initializeDevice(XsDevice* device) const;

private:


	std::map<DeviceTypeId, MasterConstructFunc> m_masterConstructors;
	std::map<DeviceTypeId, StandaloneConstructFunc> m_standaloneConstructors;

	DeviceManager *m_deviceManager;
};

namespace DeviceType {
	static const DeviceFactory::DeviceTypeId INVALID = 0;
}
#endif
