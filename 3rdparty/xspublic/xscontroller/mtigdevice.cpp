
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

#include "mtigdevice.h"
#include <xstypes/xsstringoutputtypearray.h>
#include <xstypes/xsstatusflag.h>

#define MTMK4_700_LEGACY_FW_VERSION_MAJOR		1
#define MTMK4_700_LEGACY_FW_VERSION_MINOR		3
#define MTMK4_700_LEGACY_FW_VERSION_REVISION	7

using namespace xsens;

/*! \brief An empty constructor
*/
void MtigDevice::construct()
{
}

/*! \brief Constructs a device
	\param comm The communicator to construct with
*/
MtigDevice::MtigDevice(Communicator* comm)
	: MtiBaseDeviceEx(comm)
{
	construct();
}

/*! \brief An empty constructor
	\param master The master device
*/
MtigDevice::MtigDevice(MtContainer *master)
	: MtiBaseDeviceEx(master)
{
	construct();
}

/*! \brief Destroys a device
*/
MtigDevice::~MtigDevice()
{
}

XsStringOutputTypeArray MtigDevice::supportedStringOutputTypes() const
{
	XsStringOutputTypeArray outputs;
	outputs.push_back(XSOT_HCHDM);
	outputs.push_back(XSOT_HCHDG);
	outputs.push_back(XSOT_TSS2);
	outputs.push_back(XSOT_PHTRO);
	outputs.push_back(XSOT_PRDID);
	outputs.push_back(XSOT_EM1000);
	outputs.push_back(XSOT_PSONCMS);
	outputs.push_back(XSOT_HCMTW);
	outputs.push_back(XSOT_HEHDT);
	outputs.push_back(XSOT_HEROT);
	outputs.push_back(XSOT_GPGGA);
	outputs.push_back(XSOT_PTCF);
	outputs.push_back(XSOT_XSVEL);
	outputs.push_back(XSOT_GPZDA);
	outputs.push_back(XSOT_GPRMC);

	return outputs;
}

/*! \brief Returns the base update rate (hz) corresponding to the dataType
*/
MtiBaseDevice::BaseFrequencyResult MtigDevice::getBaseFrequencyInternal(XsDataIdentifier dataType) const
{
	MtiBaseDevice::BaseFrequencyResult result;
	result.m_frequency = 0;
	result.m_divedable = true;

	if ((dataType & XDI_FullTypeMask) == XDI_AccelerationHR || (dataType & XDI_FullTypeMask) == XDI_RateOfTurnHR)
	{
		result.m_frequency = 1000;
		result.m_divedable = false;

		return result;
	}

	auto baseFreq = [&](XsDataIdentifier dataType)
	{
		XsVersion const legacyFwVersion(MTMK4_700_LEGACY_FW_VERSION_MAJOR, MTMK4_700_LEGACY_FW_VERSION_MINOR, MTMK4_700_LEGACY_FW_VERSION_REVISION);
		bool const isLegacyFirmware = (firmwareVersion() <= legacyFwVersion);

		switch (dataType & XDI_TypeMask)
		{
		case XDI_None:					return 2000;
		case XDI_TimestampGroup:		return XDI_MAX_FREQUENCY_VAL;

		case XDI_RawSensorGroup:		return 2000;
		case XDI_AnalogInGroup:			return 2000;
		case XDI_StatusGroup:			return 2000;

		case XDI_TemperatureGroup:		return 400;
		case XDI_PositionGroup:			return 400;
		case XDI_VelocityGroup:			return 400;
		case XDI_OrientationGroup:		return 400;
		case XDI_AccelerationGroup:		return 400;
		case XDI_AngularVelocityGroup:	return 400;
		case XDI_MagneticGroup:			return 100;
		case XDI_PressureGroup:			return 50;

		case XDI_GnssGroup:				return isLegacyFirmware ? 0 : 4;
		default:						return 0;
		}
	};

	result.m_frequency = baseFreq(dataType);

	if (((dataType & XDI_TypeMask) == XDI_TimestampGroup) || ((dataType & XDI_TypeMask) == XDI_GnssGroup))
		result.m_divedable = false;

	return result;
}

uint32_t MtigDevice::supportedStatusFlags() const
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
		|XSF_SyncIn
		|XSF_SyncOut
		|XSF_FilterMode
		|XSF_HaveGnssTimePulse
		);
}
