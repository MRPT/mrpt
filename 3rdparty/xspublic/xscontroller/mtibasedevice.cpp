
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

#include "mtibasedevice.h"
#include "xsdef.h"
#include "synclinemk4.h"
#include "synclinegmt.h"
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xssyncsettingarray.h>
#include <xstypes/xsportinfo.h>
#include "xsicccommand.h"
#include <xstypes/xsmatrix.h>
#include "xsiccrepmotionresult.h"
#include <xstypes/xsquaternion.h>
#include <xstypes/xsvector.h>
#include <set>
#include <xstypes/xsstatusflag.h>

using namespace xsens;

/*! \brief Constructs a device
	\param comm The communicator to construct with
*/
MtiBaseDevice::MtiBaseDevice(Communicator* comm)
	: MtDeviceEx(comm)
{
}

/*! \brief Constructs a child device for a master device
	\param master The master device to construct for
*/
MtiBaseDevice::MtiBaseDevice(MtContainer *master)
	: MtDeviceEx(master, XsDeviceId())
{
}

/*! \brief Default destructor
*/
MtiBaseDevice::~MtiBaseDevice()
{
}

/*! \copybrief XsDevice::setHeadingOffset
*/
bool MtiBaseDevice::setHeadingOffset(double)
{
	return false;
}

/*! \copybrief XsDevice::outputConfiguration
*/
XsOutputConfigurationArray MtiBaseDevice::outputConfiguration() const
{
	XsMessage snd(XMID_ReqOutputConfiguration), rcv;
	if (!doTransaction(snd, rcv))
		return XsOutputConfigurationArray();

	XsOutputConfigurationArray rv;
	XsSize count = rcv.getDataSize() >> 2;
	for (XsSize row = 0; row < count; row++)
	{
		uint16_t dataId, freq;
		dataId = rcv.getDataShort(row * 4);
		freq = rcv.getDataShort(2 + row * 4);
		rv.push_back(XsOutputConfiguration((XsDataIdentifier)dataId, freq));
	}

	return rv;
}

/*! \brief Set the output configuration for this device
	\param config The desired output configuration
	\returns true if the output configuration was successfully updated
*/
bool MtiBaseDevice::setOutputConfiguration(XsOutputConfigurationArray& config)
{
	if (!deviceId().isMti6X0())
		setStringOutputMode(0, 0, 0);

	if (!MtDeviceEx::setOutputConfiguration(config))
		return false;

	return true;
}

/*! \copybrief XsDevice::setAlignmentRotationMatrix
*/
bool MtiBaseDevice::setAlignmentRotationQuaternion(XsAlignmentFrame frame, const XsQuaternion& quat)
{
	XsMessage snd(XMID_SetAlignmentRotation, 1 + XS_LEN_ALIGNMENTROTATION);
	snd.setBusId(busId());
	snd.setDataByte((uint8_t)frame);
	for (XsSize i = 0; i < 4; ++i)
		snd.setDataFloat((float)quat[i], (uint16_t) (1 + i * sizeof(float)));

	if (!doTransaction(snd))
		return false;

	return true;
}

/*! \copybrief XsDevice::alignmentRotationQuaternion
*/
XsQuaternion MtiBaseDevice::alignmentRotationQuaternion(XsAlignmentFrame frame) const
{
	XsMessage snd(XMID_ReqAlignmentRotation), rcv;
	snd.setDataByte(frame);
	if (!doTransaction(snd, rcv))
		return XsQuaternion();

	XsQuaternion quat;
	for (int i = 0; i < 4; i++)
		quat[i] = rcv.getDataFloat(1 + i * 4);
	return quat;
}

/*! \copybrief XsDevice::setAlignmentRotationMatrix
*/
bool MtiBaseDevice::setAlignmentRotationMatrix(XsAlignmentFrame frame, const XsMatrix& matrix)
{
	return setAlignmentRotationQuaternion(frame, XsQuaternion(matrix).normalized());
}

/*! \copybrief XsDevice::alignmentRotationMatrix
*/
XsMatrix MtiBaseDevice::alignmentRotationMatrix(XsAlignmentFrame frame) const
{
	XsMatrix matrix;
	matrix.fromQuaternion(alignmentRotationQuaternion(frame));
	return matrix;
}

/*! \copybrief XsDevice::setSyncSettings
*/
bool MtiBaseDevice::setSyncSettings(const XsSyncSettingArray &s)
{
	/* Mk4 sync lines:
		* XSL_ClockIn => 0		external clock sync (SMCU)
		* XSL_GnssClockIn => 1	GNSS clock sync (DSP?)
		* XSL_In1 => 2			send data line (IMCU)
		* XSL_Bi1In => 3		SMCU
		* XSL_Bi1Out => 4		SMCU
	*/

	int timeResolution = XsDevice::syncSettingsTimeResolutionInMicroSeconds(deviceId());

	if (s.size() > 10)
		return false;

	//if no item, create the length of 1 item with zero values
	XsMessage snd(XMID_SetSyncConfiguration, (s.size() == 0) ? 12 : s.size()*12);
	XsMessage rcv;

	snd.setBusId(busId());

	/* create message but abort if we encounter anything out of the ordinary
		each sync setting is:
		0: event / action (function)
		1: line
		2: polarity
		3: triggerOnce
		4-5: skipFirst
		6-7: skipFactor
		8-9: pulse width
		10-11: delay/offset/clock frequency
	*/
	for (size_t i = 0; i < s.size(); ++i)
	{
		const XsSyncSetting& setting = s[i];
		XsSize offset = i*12;

		snd.setDataByte(setting.m_function, offset+0);
		const uint8_t line = syncLine(setting);
		snd.setDataByte(line, offset+1);
		assert(setting.m_polarity != XSP_None);
		snd.setDataByte(setting.m_polarity, offset+2);
		snd.setDataByte(setting.m_triggerOnce?1:0, offset+3);
		snd.setDataShort(setting.m_skipFirst, offset+4);
		snd.setDataShort(setting.m_skipFactor, offset+6);
		snd.setDataShort((uint16_t) (setting.m_pulseWidth / timeResolution), offset+8);
		uint16_t value = 0;
		if (setting.m_function == XSF_ClockBiasEstimation || setting.m_function == XSF_SampleAndSend)
		{
			snd.setDataShort(setting.m_clockPeriod, offset+10);
		}
		else
		{
			value = (uint16_t) (setting.m_offset / timeResolution);
			snd.setDataShort(value, offset+10);
		}
	}

	if (!doTransaction(snd))
		return false;

	return true;
}

/*! \copybrief XsDevice::syncSettings
*/
XsSyncSettingArray MtiBaseDevice::syncSettings() const
{
	XsMessage snd(XMID_SyncConfiguration), rcv;
	if (!doTransaction(snd, rcv))
		return XsSyncSettingArray();

	const uint8_t* emtsBuffer = rcv.getDataBuffer();
	return syncSettingsFromBuffer(emtsBuffer);
}

/*! \brief Create an XsSyncSttingsArray from the given buffer of sync configuration data */
XsSyncSettingArray MtiBaseDevice::syncSettingsFromBuffer(const uint8_t* buffer) const
{
	int timeResolution = XsDevice::syncSettingsTimeResolutionInMicroSeconds(deviceId());

	XsSyncSettingArray rv;
	for (int i = 0; i < 10; ++i)
	{
		XsSize offset = i*12;
		XsSyncSetting ss;
		ss.m_function = (XsSyncFunction) buffer[offset+0];
		ss.m_polarity = (XsSyncPolarity) buffer[offset+2];
		if (ss.m_polarity == XSP_None)
			break;

		ss.m_line = syncSettingsLine(buffer, offset);
		ss.m_triggerOnce = buffer[offset+3];
		ss.m_skipFirst = *(uint16_t*)&buffer[offset+4];
		ss.m_skipFactor = *(uint16_t*)&buffer[offset+6];
		ss.m_pulseWidth = ((int32_t) *(uint16_t*)&buffer[offset+8]) * timeResolution;
		if (ss.m_function == XSF_ClockBiasEstimation || ss.m_function == XSF_SampleAndSend)
			ss.m_clockPeriod = *(uint16_t*)&buffer[offset+10];
		else
			ss.m_offset = ((int32_t) *(uint16_t*)&buffer[offset+10]) * timeResolution;

		rv.push_back(ss);
	}
	return rv;

}

/*! \returns the update rate for the specified XsDataIdentifier in the configuration list or 0 if no such data available
	\param dataType The data identifier for which the update rate is requested
	\param configurations The configuration list in which the specified dataType must be looked up
*/
int MtiBaseDevice::calculateUpdateRateImp(XsDataIdentifier dataType, const XsOutputConfigurationArray& configurations) const
{
	int matchLevel = 0;
	int result = 0;

	bool groupCheck = ((dataType & XDI_TypeMask) == dataType);
	for (XsOutputConfigurationArray::const_iterator i = configurations.begin(); i != configurations.end(); ++i)
	{
		int ml = 0;
		if ((dataType & XDI_FullTypeMask) == (i->m_dataIdentifier & XDI_FullTypeMask))
		{
			if (dataType == i->m_dataIdentifier)
				ml = 3;
			else
				ml = 2;
		}
		else if (groupCheck && (dataType == (i->m_dataIdentifier & XDI_TypeMask)))
			ml = 1;

		if (ml > matchLevel)
		{
			result = i->m_frequency;
			if (ml == 3)
				break;
			matchLevel = ml;
		}
	}

	return result;
}

/*! \returns the update rate for the specified XsDataIdentifier or 0 if no such data available
	\details This function only looks at the output configuration configured in the device, not XDA calculated data
	\param dataType The data identifier to use
	\sa calculateUpdateRateImp
*/
int MtiBaseDevice::calculateUpdateRate(XsDataIdentifier dataType) const
{
	return calculateUpdateRateImp(dataType, outputConfiguration());
}

/*! \returns the base update rate (Hz) corresponding to the dataType. Returns 0 if no update rate is available
	\param dataType The data identifier to use
*/
int MtiBaseDevice::getBaseFrequency(XsDataIdentifier dataType) const
{
	return getBaseFrequencyInternal(dataType).m_frequency;
}

/*! \copybrief XsDevice::supportedUpdateRates
*/
std::vector<int> MtiBaseDevice::supportedUpdateRates(XsDataIdentifier dataType) const
{
	std::vector<int> updateRates;
	auto baseFreq = getBaseFrequencyInternal(dataType);

	if (baseFreq.m_frequency == 0)
		return updateRates;

	if (!baseFreq.m_divedable)
	{
		updateRates.push_back(baseFreq.m_frequency);
		return updateRates;
	}

	std::set<int> unsupportedUpdateRates;
	unsupportedUpdateRates.insert(500);
	unsupportedUpdateRates.insert(250);
	unsupportedUpdateRates.insert(125);

	for (int skip = 0; skip <= baseFreq.m_frequency; ++skip)
	{
		int freq = calcFrequency(baseFreq.m_frequency, skip);
		if (freq * (skip+1) == baseFreq.m_frequency) {
			if (unsupportedUpdateRates.count(freq) == 0)
				updateRates.push_back(freq);
		}
	}

	return updateRates;
}

/*! \copybrief XsDevice::setNoRotation
*/
bool MtiBaseDevice::setNoRotation(uint16_t duration)
{
	return MtDeviceEx::setNoRotation(duration);
}

/*! \copybrief XsDevice::setInitialPositionLLA
*/
bool MtiBaseDevice::setInitialPositionLLA(const XsVector& lla)
{
	uint8_t bid = busId();
	if (bid == XS_BID_INVALID || bid == XS_BID_BROADCAST || lla.size() != 3)
		return false;

	XsMessage snd(XMID_SetLatLonAlt, 3*sizeof(double));
	snd.setDataDouble(lla[0], 0);
	snd.setDataDouble(lla[1], 8);
	snd.setDataDouble(lla[2], 16);
	snd.setBusId(bid);

	return doTransaction(snd);
}

/*! \copybrief XsDevice::utcTime
*/
XsTimeInfo MtiBaseDevice::utcTime() const
{
	uint8_t bid = busId();
	if (bid == XS_BID_INVALID || bid == XS_BID_BROADCAST)
		return XsTimeInfo();

	XsMessage snd(XMID_ReqUtcTime, 0), rcv;
	snd.setBusId(bid);

	if (!doTransaction(snd, rcv))
		return XsTimeInfo();

	XsTimeInfo time;
	time.m_nano = rcv.getDataLong(0);
	time.m_year = rcv.getDataShort(4);
	time.m_month = rcv.getDataByte(6);
	time.m_day = rcv.getDataByte(7);
	time.m_hour = rcv.getDataByte(8);
	time.m_minute = rcv.getDataByte(9);
	time.m_second = rcv.getDataByte(10);
	time.m_valid = rcv.getDataByte(11);
	time.m_utcOffset = 0;

	return time;
}

/*! \copybrief XsDevice::setUtcTime
*/
bool MtiBaseDevice::setUtcTime(const XsTimeInfo& time)
{
	uint8_t bid = busId();
	if (bid == XS_BID_INVALID || bid == XS_BID_BROADCAST)
		return false;

	XsMessage snd(XMID_SetUtcTime, XS_LEN_UTCTIME);
	snd.setDataLong(time.m_nano, 0);
	snd.setDataShort(time.m_year, 4);
	snd.setDataByte(time.m_month, 6);
	snd.setDataByte(time.m_day, 7);
	snd.setDataByte(time.m_hour, 8);
	snd.setDataByte(time.m_minute, 9);
	snd.setDataByte(time.m_second, 10);
	snd.setDataByte(time.m_valid, 11);
	snd.setBusId(bid);

	if (!doTransaction(snd))
		return false;

	return true;
}

/*! \copybrief XsDevice::setErrorMode
*/
bool MtiBaseDevice::setErrorMode(XsErrorMode em)
{
	if (em == XEM_IncreasePacketCounterAndSendError)
		return true;
	return false;
}

/*! \copybrief XsDevice::errorMode
*/
XsErrorMode MtiBaseDevice::errorMode() const
{
	return XEM_IncreasePacketCounterAndSendError;
}

/*! \copybrief XsDevice::rs485TransmissionDelay
*/
uint16_t MtiBaseDevice::rs485TransmissionDelay() const
{
	XsMessage snd(XMID_ReqTransmitDelay), rcv;
	if (!doTransaction(snd, rcv))
		return 0;
	return rcv.getDataShort();
}

/*! \copybrief XsDevice::setRs485TransmissionDelay
*/
bool MtiBaseDevice::setRs485TransmissionDelay(uint16_t delay)
{
	return delay == 0;
}

/*! \copybrief XsDevice::resetRemovesPort
*/
bool MtiBaseDevice::resetRemovesPort() const
{
	Communicator* comm = communicator();
	if (!comm)
		return false;

	bool removesPort = false;
	uint16_t vid, pid;
	comm->portInfo().getVidPid(vid, pid);
	// Direct connection, COM port will be removed
	if (vid == XSENS_VENDOR_ID && pid < 0x00FF)
		removesPort = true;

	return removesPort;
}

/*! \brief Returns the sync settings line for a generic mti device
*/
XsSyncLine MtiBaseDevice::syncSettingsLine(const uint8_t* buff, XsSize offset) const
{
	if (deviceId().isMtMark4() || deviceId().isMtMark5())
		return	xsl4ToXsl((SyncLineMk4) buff[offset + 1]);
	else
		return	xslgmtToXsl((SyncLineGmt) buff[offset + 1]);
}

/*! \brief Returns the sync line for a generic mti device
*/
uint8_t MtiBaseDevice::syncLine(const XsSyncSetting& setting) const
{
	if (deviceId().isMtMark4() || deviceId().isMtMark5())
	{
		SyncLineMk4 mk4Line = xslToXsl4(setting.m_line);
		assert(mk4Line != XSL4_Invalid);
		if (mk4Line == XSL4_ClockIn || mk4Line == XSL4_GnssClockIn)
		{
			assert(setting.m_function == XSF_ClockBiasEstimation);
		}
		return static_cast<uint8_t>(mk4Line);
	}
	else
	{
		SyncLineGmt gmtLine = xslToXslgmt(setting.m_line);
		assert(gmtLine != XSLGMT_Invalid);
		if (gmtLine == XSLGMT_ClockIn)
		{
			assert(setting.m_function == XSF_SampleAndSend);
		}
		return static_cast<uint8_t>(gmtLine);
	}
}

bool MtiBaseDevice::messageLooksSane(const XsMessage &msg) const
{
	(void)msg;
	return true;
}

/*! \returns True if device uses on board filtering */
bool MtiBaseDevice::deviceUsesOnBoardFiltering()
{
	return updateRateForDataIdentifier(XDI_OrientationGroup) > 0;
}

/*! \brief Starts the representative motion
	\returns True if successful
*/
bool MtiBaseDevice::startRepresentativeMotion()
{
	if (!hasIccSupport())
		return false;

	if (!deviceUsesOnBoardFiltering())
		return false;

	XsMessage snd(XMID_IccCommand, 1);
	snd.setBusId(busId());
	snd.setDataByte(XIC_StartRepMotion);

	if (!doTransaction(snd))
		return false;

	return true;
}

/*! \returns True if the representative motion is running */
bool MtiBaseDevice::representativeMotionState()
{
	if (!hasIccSupport())
		return false;

	if (!deviceUsesOnBoardFiltering())
		return false;

	bool enabled = false;

	XsMessage snd(XMID_IccCommand, 1);
	snd.setBusId(busId());
	snd.setDataByte(XIC_RepMotionState);
	XsMessage rcv;
	if (doTransaction(snd, rcv))
		enabled = (rcv.getDataByte(1) != 0);

	return enabled;
}

/*! \brief Stops the representative motion
	\returns The results of the representative motion
*/
XsIccRepMotionResult MtiBaseDevice::stopRepresentativeMotion()
{
	XsIccRepMotionResult result;

	if (!hasIccSupport())
		return result;

	if (!deviceUsesOnBoardFiltering())
		return result;

	XsMessage snd(XMID_IccCommand, 1), rcv;
	snd.setBusId(busId());
	snd.setDataByte(XIC_StopRepMotion);

	if (doTransaction(snd, rcv, 3500))
	{
		result.m_ddtAccuracy = rcv.getDataFloat(1);
		result.m_dimension = rcv.getDataByte(5);
		result.m_status = rcv.getDataByte(6);
	}

	return result;
}

/*! \brief Stores the ICC results
	\returns True if successful
*/
bool MtiBaseDevice::storeIccResults()
{
	if (!deviceUsesOnBoardFiltering())
		return false;

	XsMessage snd(XMID_IccCommand, 1);
	snd.setBusId(busId());
	snd.setDataByte(XIC_StoreResults);

	return doTransaction(snd, 2000);
}

/*! \returns True if this device has an ICC support
*/
bool MtiBaseDevice::hasIccSupport() const
{
	return (firmwareVersion() >= XsVersion(1, 5, 0));
}

void MtiBaseDevice::fetchAvailableHardwareScenarios()
{
	if (deviceId().isImu())						// If we are a 100 type device,
		m_hardwareFilterProfiles.clear();				// there are no filter profiles in the firmware.
	else												// For other device types,
		MtDeviceEx::fetchAvailableHardwareScenarios();	// fetch the scenarios.
}
