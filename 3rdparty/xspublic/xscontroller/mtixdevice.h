
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

#ifndef XSMTIXDEVICE_H
#define XSMTIXDEVICE_H

#include "mtibasedevice.h"

class MtContainer;

/*! \class MtiXDevice
	\brief The MTi device used for the X-series
*/
class MtiXDevice : public MtiBaseDeviceEx
{
public:
	/*! \brief Constructs a standalone device using a provided communicator
		\param comm The communicator to use
		\return The constructed device
	*/
	static XsDevice* constructStandalone(Communicator* comm)
	{
		return new MtiXDevice(comm);
	}

	explicit MtiXDevice(Communicator* comm);

	//! \brief An empty constructor for a master device
	explicit MtiXDevice(MtContainer *master) : MtiBaseDeviceEx(master) {}
	virtual ~MtiXDevice();

	uint32_t supportedStatusFlags() const override;

protected:
	bool hasIccSupport() const override;

	MtiBaseDevice::BaseFrequencyResult getBaseFrequencyInternal(XsDataIdentifier dataType = XDI_None) const override;
};

#ifndef XDA_PRIVATE_BUILD
/*! \class MtiXDeviceEx
	\brief The internal base class for MTi-X series devices
*/
struct MtiXDeviceEx : public MtiXDevice
{
	//! \copybrief MtiXDevice::MtiXDevice
	explicit MtiXDeviceEx(Communicator* comm) : MtiXDevice(comm) {};

	//! \copybrief MtiXDevice::MtiXDevice
	explicit MtiXDeviceEx(MtContainer *master) : MtiXDevice(master) {};
};
#else
#include "mtixdeviceex.h"
#endif

#endif
