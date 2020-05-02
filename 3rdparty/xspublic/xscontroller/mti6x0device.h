
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

#ifndef XSMTI6X0DEVICE_H
#define XSMTI6X0DEVICE_H

#include "mtibasedevice.h"

class MtContainer;

/*! \class Mti6X0Device
	\brief The MTi device used for the 6X0-series
*/
class Mti6X0Device : public MtiBaseDeviceEx
{
public:
	//! \copybrief MtiXDevice::constructStandalone
	static XsDevice* constructStandalone(Communicator* comm)
	{
		return new Mti6X0Device(comm);
	}

	explicit Mti6X0Device(Communicator* comm);

	//! \brief An empty constructor for a master device
	explicit Mti6X0Device(MtContainer *master) : MtiBaseDeviceEx(master) {}
	virtual ~Mti6X0Device();

	XsStringOutputTypeArray supportedStringOutputTypes() const override;
	bool setStringOutputMode(uint32_t type, uint16_t frequency);
	uint32_t supportedStatusFlags() const override;
	XsCanOutputConfigurationArray canOutputConfiguration() const override;
	bool setCanOutputConfiguration(XsCanOutputConfigurationArray& config) override;

	XsIntArray portConfiguration() const override;
	bool setPortConfiguration(XsIntArray& config) override;

	uint32_t canConfiguration() const override;
	bool setCanConfiguration(uint32_t config) override;

protected:
	uint8_t syncLine(const XsSyncSetting& setting) const override;
	bool hasIccSupport() const override;

	MtiBaseDevice::BaseFrequencyResult getBaseFrequencyInternal(XsDataIdentifier dataType = XDI_None) const override;
};

#ifndef XDA_PRIVATE_BUILD
/*! \class Mti6X0DeviceEx
	\brief The internal base class for MTi-6X0 series devices
*/
struct Mti6X0DeviceEx : public Mti6X0Device
{
	//! \copybrief Mti6X0Device::Mti6X0Device
	explicit Mti6X0DeviceEx(Communicator* comm) : Mti6X0Device(comm) {};

	//! \copybrief Mti6X0Device::Mti6X0Device
	explicit Mti6X0DeviceEx(MtContainer *master) : Mti6X0Device(master) {};
};
#else
#include "mtix00deviceex.h"
#endif

#endif
