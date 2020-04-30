
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

#include "mti6x0device.h"

#include <xstypes/xsdatapacket.h>
#include "replyobject.h"
#include "communicator.h"
#include "scenariomatchpred.h"
#include "messageserializer.h"
#include <xstypes/xsstatusflag.h>

#include <xstypes/xssensorranges.h>
#include "synclinegmt.h"
#include <xstypes/xssyncsetting.h>
#include <xstypes/xscanoutputconfigurationarray.h>
#include <xstypes/xsstringoutputtypearray.h>
#include <xstypes/xsintarray.h>

using namespace xsens;

/*! \brief Constructs a device
	\param comm The communicator to construct with
*/
Mti6X0Device::Mti6X0Device(Communicator* comm) :
	MtiBaseDeviceEx(comm)
{
}

/*! \brief Destroys a device
*/
Mti6X0Device::~Mti6X0Device()
{
}

/*! \brief Returns the base update rate (Hz) corresponding to the dataType
*/
MtiBaseDevice::BaseFrequencyResult Mti6X0Device::getBaseFrequencyInternal(XsDataIdentifier dataType) const
{
	MtiBaseDevice::BaseFrequencyResult result;
	result.m_frequency = 0;
	result.m_divedable = true;

	if ((dataType == XDI_FreeAcceleration && deviceId().isImu()) ||
		((dataType & XDI_FullTypeMask) == XDI_LocationId) ||
		((dataType & XDI_FullTypeMask) == XDI_DeviceId) ||
		(dataType == XDI_GnssSatInfo))
		return result;

	if ((dataType & XDI_FullTypeMask) == XDI_AccelerationHR)
	{
		result.m_frequency = 2000;
		return result;
	}

	if ((dataType & XDI_FullTypeMask) == XDI_RateOfTurnHR)
	{
		result.m_frequency = 1600;
		return result;
	}

	auto baseFreq = [&](XsDataIdentifier dataType)
	{
		switch (dataType & XDI_TypeMask)
		{
		case XDI_None:					return 400;
		case XDI_TimestampGroup:		return XDI_MAX_FREQUENCY_VAL;
		case XDI_StatusGroup:			return 400;
		case XDI_TemperatureGroup:		return 400;
		case XDI_OrientationGroup:		return deviceId().isImu() ? 0 : 400;
		case XDI_AccelerationGroup:		return 400;
		case XDI_AngularVelocityGroup:	return 400;
		case XDI_MagneticGroup:			return 100;

		case XDI_GnssGroup:				return deviceId().isGnss() ? 4 : 0;
		case XDI_PressureGroup:			return 100;
		case XDI_PositionGroup:			return deviceId().isGnss() ? 400 : 0;
		case XDI_VelocityGroup:			return deviceId().isGnss() ? 400 : 0;
		default:						return 0;
		}
	};
	result.m_frequency = baseFreq(dataType);

	if (((dataType & XDI_TypeMask) == XDI_TimestampGroup) || ((dataType & XDI_TypeMask) == XDI_GnssGroup))
		result.m_divedable = false;

	return result;
}

/*! \returns The sync line for a mtitx0 device This overrides the base class method.
	\param setting The sync setting to get a sync line from
*/
uint8_t Mti6X0Device::syncLine(const XsSyncSetting& setting) const
{
	SyncLineGmt gmtLine = xslToXslgmt(setting.m_line);
	assert(gmtLine != XSLGMT_Invalid);
	return static_cast<uint8_t>(gmtLine);
}

/*! \returns True if this device has an ICC support
*/
bool Mti6X0Device::hasIccSupport() const
{
	return (firmwareVersion() >= XsVersion(1, 1, 0));
}


XsStringOutputTypeArray Mti6X0Device::supportedStringOutputTypes() const
{
	XsStringOutputTypeArray outputs;

	outputs.push_back(XSOT_PSONCMS);
	outputs.push_back(XSOT_HCMTW);
	outputs.push_back(XSOT_HEROT);
	outputs.push_back(XSOT_PTCF);
	outputs.push_back(XSOT_GPZDA);
	outputs.push_back(XSOT_TSS2);
	outputs.push_back(XSOT_PHTRO);
	outputs.push_back(XSOT_PRDID);
	outputs.push_back(XSOT_EM1000);
	outputs.push_back(XSOT_HEHDT);
	outputs.push_back(XSOT_HCHDM);
	outputs.push_back(XSOT_GPGGA);
	outputs.push_back(XSOT_GPRMC);
	outputs.push_back(XSOT_XSVEL);
	outputs.push_back(XSOT_HCHDG);

	return outputs;
}

/*! \brief Sets the string output mode for this device
	\param type The type to set
	\param frequency The frequency to set
	\returns True if the device was successfully updated
*/
bool Mti6X0Device::setStringOutputMode(uint32_t type, uint16_t frequency)
{
	XsMessage sndType(XMID_SetStringOutputConfig);
	sndType.setBusId(XS_BID_MASTER);	// Always send to master device
	sndType.setDataLong(type);
	sndType.setDataShort(frequency, 4);

	if (!doTransaction(sndType))
		return false;

	return true;
}

uint32_t Mti6X0Device::supportedStatusFlags() const
{
	return (uint32_t) (0
		| (deviceId().isImu() ? 0 : XSF_OrientationValid
			|XSF_NoRotationMask
			|XSF_RepresentativeMotion
			)
		|XSF_ExternalClockSynced
		| (deviceId().isGnss() ? XSF_GpsValid : 0)
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
		| (deviceId().isGnss() ? XSF_FilterMode : 0)
		| (deviceId().isGnss() ? XSF_HaveGnssTimePulse : 0)
		);
}

/*! \copybrief XsDevice::canOutputConfiguration
*/
XsCanOutputConfigurationArray Mti6X0Device::canOutputConfiguration() const
{
	XsMessage snd(XMID_ReqCanConfig), rcv;
	if (!doTransaction(snd, rcv))
		return XsCanOutputConfigurationArray();

	XsCanOutputConfigurationArray config;
	MessageDeserializer serializer(rcv);
	serializer >> config;

	return config;
}

/*! \copydoc XsDevice::setCanOutputConfiguration
*/
bool Mti6X0Device::setCanOutputConfiguration(XsCanOutputConfigurationArray& config)
{
	XsMessage snd(XMID_SetCanOutputConfig, 4);
	snd.setBusId((uint8_t)busId());
	bool wasEmpty = config.empty();
	MessageSerializer(snd) << config;

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	MessageDeserializer(rcv) >> config;
	if (wasEmpty && config.size() == 1 && config[0] == XsCanOutputConfiguration(XCFF_11Bit_Identifier, XCDI_Invalid, 0, 0))
		config.clear();
	return true;
}

/*! \copybrief XsDevice::canConfiguration
*/
uint32_t Mti6X0Device::canConfiguration() const
{
	XsMessage snd(XMID_ReqCanConfig), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataLong();
}

/*! \copydoc XsDevice::portConfiguration
*/
XsIntArray Mti6X0Device::portConfiguration() const
{
	XsMessage snd(XMID_ReqPortConfig), rcv;
	if (!doTransaction(snd, rcv))
		return XsIntArray();

	XsIntArray rv;
	rv.push_back((int)rcv.getDataLong(0));
	rv.push_back((int)rcv.getDataLong(4));
	rv.push_back((int)rcv.getDataLong(8));
	return rv;
}

/*! \copydoc XsDevice::setPortConfiguration
*/
bool Mti6X0Device::setPortConfiguration(XsIntArray &config)
{
	XsIntArray currentConfig = portConfiguration();

	XsMessage snd(XMID_SetPortConfig);
	snd.setBusId((uint8_t)busId());

	if (config.size() > 0)
		snd.setDataLong((uint32_t)config[0], 0);
	if (config.size() > 1)
		snd.setDataLong((uint32_t)config[1], 4);
	if (config.size() > 2)
		snd.setDataLong((uint32_t)config[2], 8);

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	if (currentConfig != config)
		return reset();

	return true;
}

/*! \copydoc XsDevice::setCanOutputConfiguration
*/
bool Mti6X0Device::setCanConfiguration(uint32_t config)
{
	uint32_t currentConfig = canConfiguration();

	XsMessage snd(XMID_SetCanConfig, 4);
	snd.setBusId((uint8_t)busId());
	snd.setDataLong(config);

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	if ((currentConfig & 0xFF) != (config & 0xFF))
		return reset();

	return true;
}
