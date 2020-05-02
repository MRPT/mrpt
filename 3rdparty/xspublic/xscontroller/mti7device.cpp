
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

#include "mti7device.h"
#include <xstypes/xsstatusflag.h>

void Mti7Device::construct()
{
}

Mti7Device::Mti7Device(Communicator* comm)
	: MtiBaseDeviceEx(comm)
{
	construct();

	if (comm)
		comm->setDefaultTimeout(1000); //Increase the default timeout for MTi-1 devices because a settings write can occasionally take ~900ms
}

Mti7Device::Mti7Device(MtContainer *masterdevice)
	: MtiBaseDeviceEx(masterdevice)
{
	construct();
}

Mti7Device::~Mti7Device()
{
}


/*! \brief Returns the base update rate (hz) corresponding to the dataType
*/
MtiBaseDevice::BaseFrequencyResult Mti7Device::getBaseFrequencyInternal(XsDataIdentifier dataType) const
{
	BaseFrequencyResult result;
	result.m_frequency = 0;
	result.m_divedable = true;

	if ((dataType & XDI_FullTypeMask) == XDI_LocationId || (dataType & XDI_FullTypeMask) == XDI_DeviceId)
		return result;

	if ((dataType & XDI_FullTypeMask) == XDI_AccelerationHR || (dataType & XDI_FullTypeMask) == XDI_RateOfTurnHR)
	{
		result.m_frequency = 800;
		result.m_divedable = true;

		return result;
	}

	auto baseFreq = [&](XsDataIdentifier dataType)
	{
		switch (dataType & XDI_TypeMask)
		{
		case XDI_None:					return 100;
		case XDI_TimestampGroup:		return XDI_MAX_FREQUENCY_VAL;
		case XDI_StatusGroup:			return 100;
		case XDI_TemperatureGroup:		return 100;
		case XDI_PositionGroup:			return 100;
		case XDI_VelocityGroup:			return 100;
		case XDI_OrientationGroup:		return 100;
		case XDI_AccelerationGroup:		return 100;
		case XDI_AngularVelocityGroup:	return 100;
		case XDI_MagneticGroup:			return 100;
		case XDI_PressureGroup:			return 50;
		case XDI_GnssGroup:
		{
			if ((dataType & XDI_FullTypeMask) == XDI_GnssPvtData)
				return 4;
			return 0;
		}
		default:						return 0;
		}
	};

	result.m_frequency = baseFreq(dataType);

	if (((dataType & XDI_TypeMask) == XDI_TimestampGroup) || ((dataType & XDI_TypeMask) == XDI_GnssGroup))
		result.m_divedable = false;

	return result;
}

bool Mti7Device::hasIccSupport() const
{
	return true;
}

uint32_t Mti7Device::supportedStatusFlags() const
{
	return (uint32_t) (0
		//|XSF_SelfTestOk
		|XSF_OrientationValid
		|XSF_GpsValid
		|XSF_NoRotationMask
		|XSF_RepresentativeMotion
		|XSF_ExternalClockSynced
		|XSF_ClipAccX
		|XSF_ClipAccY
		|XSF_ClipAccZ
		|XSF_ClipGyrX
		|XSF_ClipGyrY
		|XSF_ClipGyrZ
		|XSF_ClipMagX
		|XSF_ClipMagY
		|XSF_ClipMagZ
		//|XSF_Retransmitted
		|XSF_ClippingDetected
		//|XSF_Interpolated
		//|XSF_SyncIn
		//|XSF_SyncOut
		|XSF_FilterMode
		|XSF_HaveGnssTimePulse
		);
}
