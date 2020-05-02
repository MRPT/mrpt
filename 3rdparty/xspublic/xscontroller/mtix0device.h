
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

#ifndef XSMTIX0DEVICE_H
#define XSMTIX0DEVICE_H

#include "mtibasedevice.h"

class MtContainer;

/*! \class MtiX0Device
	\brief The MTi device used for the X0-series
*/
class MtiX0Device : public MtiBaseDeviceEx
{
public:
	//! \copybrief MtiXDevice::constructStandalone
	static XsDevice* constructStandalone(Communicator* comm)
	{
		return new MtiX0Device(comm);
	}

	explicit MtiX0Device(Communicator* comm);

	//! \brief An empty constructor for a master device
	explicit MtiX0Device(MtContainer *master) : MtiBaseDeviceEx(master) {};
	virtual ~MtiX0Device();

	XsStringOutputTypeArray supportedStringOutputTypes() const override;
	uint32_t supportedStatusFlags() const override;

protected:
	MtiBaseDevice::BaseFrequencyResult getBaseFrequencyInternal(XsDataIdentifier dataType = XDI_None) const override;
};

#ifndef XDA_PRIVATE_BUILD
/*! \class MtiX0DeviceEx
	\brief The internal base class for MTi-X0 series devices
*/
struct MtiX0DeviceEx : public MtiX0Device
{
	//! \copybrief MtiX0Device::MtiX0Device
	explicit MtiX0DeviceEx(Communicator* comm) : MtiX0Device(comm) {};

	//! \copybrief MtiX0Device::MtiX0Device
	explicit MtiX0DeviceEx(MtContainer *master) : MtiX0Device(master) {};
};
#else
#include "mtix0deviceex.h"
#endif

#endif
