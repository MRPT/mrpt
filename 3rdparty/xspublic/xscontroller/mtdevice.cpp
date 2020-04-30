
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

#include "mtdevice.h"
#include "xsdef.h"
#include <xstypes/xssensorranges.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsvector3.h>
#include "xsselftestresult.h"
#include <xstypes/xsstatusflag.h>

using namespace xsens;

/*! \brief Constructs a standalone MtDevice based on \a comm
*/
MtDevice::MtDevice(Communicator* comm)
	: XsDeviceEx(comm)
{
}

/*! \brief Constructs a standalone MtDevice based on \a master and \a childDeviceId
*/
MtDevice::MtDevice(MtContainer * master, const XsDeviceId &childDeviceId)
	: XsDeviceEx(master, childDeviceId)
{
}

/*! \brief Destroys the MtDevice
*/
MtDevice::~MtDevice()
{
	JLTRACEG("entry");
	XSEXITLOGN(gJournal);
}
/*! \brief Checks for the sanity of a message
	\param msg A message to check
	\returns True if successful
*/
bool MtDevice::messageLooksSane(const XsMessage &msg) const
{
	return msg.getBusId() == 1 || XsDevice::messageLooksSane(msg);
}

/*! \brief Initialize the Mt device using the supplied filter profiles
	\returns True if successful
*/
bool MtDevice::initialize()
{
	if (!XsDeviceEx::initialize())
		return false;

	// we must create the data caches first so they are available even if the rest of the init fails
	// when reading from file almost all init can fail but we can still read data into the caches
	if (!readDeviceConfiguration())
	{
		setInitialized(false);
		return false;
	}

	fetchAvailableHardwareScenarios();
	MtDevice::updateFilterProfiles();

	return true;
}

/*! \brief Updates the scenarios
*/
void MtDevice::updateFilterProfiles()
{
	const XsMtDeviceConfiguration& info = deviceConfigurationConst().deviceInfo(deviceId());
	if (info.m_filterProfile != 0)
	{
		m_hardwareFilterProfile = XsFilterProfile(info.m_filterProfile & 0xFF
			, info.m_filterProfile >> 8
			, m_hardwareFilterProfile.kind()
			, m_hardwareFilterProfile.label()
			, info.m_filterType
			, info.m_filterMajor
			, info.m_filterMinor);
	}

	for (auto i = m_hardwareFilterProfiles.begin(); i != m_hardwareFilterProfiles.end(); ++i)
	{
		if (i->type() == m_hardwareFilterProfile.type() || i->label() == m_hardwareFilterProfile.label())
		{
			m_hardwareFilterProfile.setLabel(i->label());
			m_hardwareFilterProfile.setKind(i->kind());
			m_hardwareFilterProfile.setVersion(i->version());
			break;
		}
	}
}

/*! \returns True if this is a motion tracker
*/
bool MtDevice::isMotionTracker() const
{
	return true;
}

/*! \copybrief XsDevice::updateRateForDataIdentifier
*/
int MtDevice::updateRateForDataIdentifier(XsDataIdentifier dataType) const
{
	return XsDevice::updateRateForDataIdentifier(dataType);
}

/*! \copybrief XsDevice::stringOutputType
*/
uint16_t MtDevice::stringOutputType() const
{
	XsMessage snd(XMID_ReqStringOutputType), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::stringSamplePeriod
*/
uint16_t MtDevice::stringSamplePeriod() const
{
	XsMessage snd(XMID_ReqPeriod), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::stringSkipFactor
*/
uint16_t MtDevice::stringSkipFactor() const
{
	XsMessage snd(XMID_ReqOutputSkipFactor), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::deviceOptionFlags
*/
XsDeviceOptionFlag MtDevice::deviceOptionFlags() const
{
	XsMessage snd(XMID_ReqOptionFlags), rcv;
	if (doTransaction(snd, rcv))
		return (XsDeviceOptionFlag)rcv.getDataLong();
	return XDOF_None;
}
/*! \copybrief XsDevice::gnssPlatform
*/
XsGnssPlatform MtDevice::gnssPlatform() const
{
	XsMessage snd(XMID_ReqGnssPlatform), rcv;
	if (doTransaction(snd, rcv))
		return (XsGnssPlatform)rcv.getDataShort();
	return XGP_Portable;
}

/*! \copybrief XsDevice::outputConfiguration
*/
XsOutputConfigurationArray MtDevice::outputConfiguration() const
{
	return XsOutputConfigurationArray();
}

/*! \brief Checks if this device can do orientation reset in firmware
	\param method The reset method
	\returns True if successful
*/
bool MtDevice::canDoOrientationResetInFirmware(XsResetMethod method)
{
	switch (method)
	{
	case XRM_DefaultAlignment:
	case XRM_DefaultHeading:
	case XRM_DefaultInclination:
		return true;

	case XRM_None:
		return false;

	default:
		break;
	}

	return updateRateForDataIdentifier(XDI_OrientationGroup) > 0;
}

/*! \copybrief XsDevice::scheduleOrientationReset */
bool MtDevice::scheduleOrientationReset(XsResetMethod method)
{
	switch (deviceState()) {
	case XDS_Measurement:
	case XDS_Recording:
		if (method == XRM_StoreAlignmentMatrix)
			return false;

		if (canDoOrientationResetInFirmware(method))
			if (!XsDevice::scheduleOrientationReset(method))
				return false;

		break;

	case XDS_Config:
		if (method != XRM_StoreAlignmentMatrix)
			return false;

		if (canDoOrientationResetInFirmware(method))
		{
			if (!storeAlignmentMatrix())
				return false;
			// read stored value from emts by reinitializing
			return reinitialize();
		}
		return true;

	default:
		return false;
	}
	return true;
}

/*! \brief Store the current alignment matrix in the device.
	\details The alignment matrix is computed when doing an orientation reset and needs to be stored
	explicitly in the device or it will be forgotten when the device restarts. This function will tell
	the device to store its alignment matrix or to write the locally computed alignment matrix to the
	device when filtering is done on the PC side.
	\returns true if the alignment matrix was successfully written to the non-volatile memory of the
	device
*/
bool MtDevice::storeAlignmentMatrix()
{
	if (!XsDevice::scheduleOrientationReset(XRM_StoreAlignmentMatrix))
		return false;

	return true;
}

/*! \brief The heading offset set for this device
*/
double MtDevice::headingOffset() const
{
	XsMessage snd(XMID_ReqHeading), rcv;
	snd.setBusId((uint8_t)busId());

	if (!doTransaction(snd, rcv))
		return 0;

	return (double)rcv.getDataFloat();
}

/*! \copybrief XsDevice::setLocationId
*/
bool MtDevice::setLocationId(int id)
{
	XsMessage snd(XMID_SetLocationId, XS_LEN_LOCATIONID);
	snd.setBusId((uint8_t)busId());
	snd.setDataShort((uint16_t)id);

	return doTransaction(snd);
}

/*! \copybrief XsDevice::locationId
*/
int MtDevice::locationId() const
{
	XsMessage snd(XMID_ReqLocationId), rcv;
	snd.setBusId((uint8_t)busId());

	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \copybrief XsDevice::serialBaudRate
*/
XsBaudRate MtDevice::serialBaudRate() const
{
	XsMessage snd(XMID_ReqBaudrate), rcv;
	snd.setBusId((uint8_t)busId());

	if (!doTransaction(snd, rcv))
		return XBR_Invalid;

	return (XsBaudRate)rcv.getDataByte();
}

/*! \copybrief XsDevice::hardwareVersion
*/
XsVersion MtDevice::hardwareVersion() const
{
	XsMessage snd(XMID_ReqHardwareVersion), rcv;
	if (!doTransaction(snd, rcv))
		return XsVersion();
	uint16_t hwv = rcv.getDataShort();
	return XsVersion(hwv >> 8, hwv & 0xff);
}

/*! \copybrief XsDevice::availableOnboardFilterProfiles
*/
XsFilterProfileArray MtDevice::availableOnboardFilterProfiles() const
{
	return m_hardwareFilterProfiles;
}

/*!	\brief Request the filter profiles headers from the hardware device and returns a vector with the found profiles.
	the order in the output vector is the same as the order in the hardware device.
*/
XsFilterProfileArray MtDevice::readFilterProfilesFromDevice() const
{
	XsFilterProfileArray result;

	XsMessage snd(XMID_ReqAvailableFilterProfiles);
	snd.setBusId((uint8_t)busId());

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return result;

	const char filterType = deviceConfigurationConst().deviceInfo(deviceId()).m_filterType;

	XsSize nofScenarios = rcv.getDataSize() / (1 + 1 + XS_LEN_FILTERPROFILELABEL);

	result.resize(nofScenarios);
	for (XsSize i = 0; i < nofScenarios; ++i)
	{
		uint8_t type = rcv.getDataByte(0 + i*(1+1+XS_LEN_FILTERPROFILELABEL));
		result[i].setType(type);
		result[i].setVersion(rcv.getDataByte(1 + i*(1+1+XS_LEN_FILTERPROFILELABEL)));
		result[i].setLabel((const char*) rcv.getDataBuffer(2 + i*(1+1+XS_LEN_FILTERPROFILELABEL)));
		result[i].setFilterType(filterType);
		XsString kind;
		if (type == XFPK_Base)
			kind = "base";
		else if (type == XFPK_Additional)
			kind = "additional";
		else if (type == XFPK_Heading)
			kind = "heading";
		result[i].setKind(kind.c_str());
	}
	return result;
}

/*! \brief Fetches available hardware scenarios
*/
void MtDevice::fetchAvailableHardwareScenarios()
{
	m_hardwareFilterProfiles.clear();
	m_hardwareFilterProfiles = readFilterProfilesFromDevice();
	std::sort(m_hardwareFilterProfiles.begin(), m_hardwareFilterProfiles.end(),
		[](XsFilterProfile const& left, XsFilterProfile const& right)
		{
			if (left.type() == right.type())
				return strcmp(left.label(), right.label()) < 0;
			else
				return left.type() < right.type();
		}
		);
}

/*! \copybrief XsDevice::productCode
*/
XsString MtDevice::productCode() const
{
	XsMessage snd(XMID_ReqProductCode), rcv;
	if (!doTransaction(snd, rcv))
		return XsString();

	const char* pc = (const char*) rcv.getDataBuffer();
	assert(pc);
	std::string result(pc?pc:"                    ", 20);
	std::string::difference_type thingy = (std::string::difference_type) result.find(" ");
	if (thingy < 20)
		result.erase(result.begin() + thingy, result.end());
	return XsString(result);
}

/*! \copybrief XsDevice::reinitialize
*/
bool MtDevice::reinitialize()
{
	if (!readDeviceConfiguration())
		return false;

	clearDataCache();
	fetchAvailableHardwareScenarios();
	return true;
}

/*! \brief Restore to factory default settings
*/
bool MtDevice::restoreFactoryDefaults()
{
	if (!XsDevice::restoreFactoryDefaults())
		return false;

	return reinitialize();
}

/*! \copybrief XsDevice::onboardFilterProfile
*/
XsFilterProfile MtDevice::onboardFilterProfile() const
{
	return m_hardwareFilterProfile;
}

/*! \copybrief XsDevice::setOnboardFilterProfile
*/
bool MtDevice::setOnboardFilterProfile(int profileType)
{
	if (deviceState() != XDS_Config)
		return false;

	XsFilterProfileArray::iterator item = std::find_if(m_hardwareFilterProfiles.begin(), m_hardwareFilterProfiles.end(),
		[profileType](XsFilterProfile const& p)
	{
		return p.type() == profileType;
	});
	if (item == m_hardwareFilterProfiles.end())
		return false;

	XsMessage snd(XMID_SetFilterProfile, XS_LEN_SETFILTERPROFILE);
	snd.setBusId((uint8_t)busId());
	snd.setDataShort((uint16_t)profileType);

	if (!doTransaction(snd))
		return false;

	m_hardwareFilterProfile = *item;
	return true;
}

/*! \copybrief XsDevice::setOnboardFilterProfile
*/
bool MtDevice::setOnboardFilterProfile(XsString const& profile)
{
	if (deviceState() != XDS_Config)
		return false;

	XsStringArray profileList(profile, "/");

	XsFilterProfileArray::iterator item[2];
	int i = 0;
	for (auto currentProfile : profileList)
	{
		item[i++] = std::find_if(m_hardwareFilterProfiles.begin(), m_hardwareFilterProfiles.end(),
			[currentProfile](XsFilterProfile const& p)
			{
				return currentProfile == p.label();
			});
		if (i == 2)
			break;
	}
	if (i == 0 || item[0] == m_hardwareFilterProfiles.end())
		return false;

	XsMessage snd(XMID_SetFilterProfile, profile.size());
	snd.setBusId((uint8_t)busId());
	snd.setDataBuffer((const uint8_t*)profile.c_str(), profile.size());

	if (!doTransaction(snd))
		return false;

	if (item[1] != m_hardwareFilterProfiles.end())
	{
		m_hardwareFilterProfile = *item[0];
		m_hardwareFilterProfile.setLabel(profile.c_str());	// Use info from first item, but label can be 2 profiles
	}
	else
		m_hardwareFilterProfile = *item[0];
	return true;
}

/*! \returns The accelerometer range for this device
	\details The range is an absolute maximum number. This means that if 100 is returned, the sensor range is (100, -100)
*/
double MtDevice::accelerometerRange() const
{
	return ::accelerometerRange(productCode(), hardwareVersion().major());
}

/*! \returns The accelerometer range for this device
	\details The range is an absolute maximum number. This means that if 300 is returned, the sensor range is (300, -300)
*/
double MtDevice::gyroscopeRange() const
{
	return ::gyroscopeRange(productCode());
}

/*! \brief Write the emts of the device to the open logfile
	\note The default implementation of MtDevice does not include any children
*/
void MtDevice::writeDeviceSettingsToFile()
{
	writeMessageToLogFile(m_emtsBlob);
}

/*! \copybrief XsDevice::setNoRotation
*/
bool MtDevice::setNoRotation(uint16_t duration)
{
	XsMessage snd(XMID_SetNoRotation, 2);
	snd.setBusId((uint8_t)busId());
	snd.setDataShort(duration);

	return doTransaction(snd);
}

/*!	\brief Set the current sensor position.
	\details Use this function to set the current position in latitude, longitude, altitude.
	\param lla The LLA vector
	\returns True if successful
	\sa initialPositionLLA
*/
bool MtDevice::setInitialPositionLLA(const XsVector& lla)
{
	uint8_t bid = (uint8_t)busId();
	if (bid == XS_BID_INVALID || bid == XS_BID_BROADCAST || lla.size() != 3)
		return false;

	XsMessage snd(XMID_SetLatLonAlt, XS_LEN_LATLONALT);
	snd.setDataFloat((float) lla[0], 0);
	snd.setDataFloat((float) lla[1], 4);
	snd.setDataFloat((float) lla[2], 8);
	snd.setBusId(bid);

	return doTransaction(snd);
}

/*! \returns the current sensor position
	\sa setInitialPositionLLA
*/
XsVector MtDevice::initialPositionLLA() const
{
	XsMessage snd(XMID_ReqLatLonAlt), rcv;
	if (doTransaction(snd, rcv))
	{
		XsVector3 vec;
		for (XsSize i = 0; i < 3; i++)
			vec[i] = rcv.getDataDouble(i * 8);
		return vec;
	}
	return XsVector();
}

/*! \brief Convert mt sync ticks to microseconds
*/
uint32_t MtDevice::syncTicksToUs(uint32_t ticks) const
{
	return ((uint32_t) (((double) ticks) * XS_SYNC_CLOCK_TICKS_TO_US + 0.5));
}

/*! \brief Convert microseconds to mt sync ticks
*/
uint32_t MtDevice::usToSyncTicks(uint32_t us) const
{
	return ((uint32_t) (((double) us) * XS_SYNC_CLOCK_US_TO_TICKS + 0.5));
}

/*! \returns the error mode of the device.
	\see setErrorMode
*/
XsErrorMode MtDevice::errorMode() const
{
	XsMessage snd(XMID_ReqErrorMode), rcv;
	if (!doTransaction(snd, rcv))
		return XEM_Ignore;
	return static_cast<XsErrorMode>(rcv.getDataShort());
}

/*! \brief Set the error mode of the device
	\details The error mode determines how the device handles errors. See
	the low-level communication documentation for more details.
	\param em The error mode
	\returns True if successful
	\see errorMode
*/
bool MtDevice::setErrorMode(XsErrorMode em)
{
	XsMessage snd(XMID_SetErrorMode, 2);
	snd.setBusId((uint8_t)busId());
	snd.setDataShort(em);
	return doTransaction(snd);
}

/*! \brief Return the RS485 acknowledge transmission delay of the device
	\details The RS485 acknowledge transmission delay determines the minimal
	delay to response on request messages. See the low-level communication
	documentation for more details.
	\returns The delay value
	\see setRs485TransmissionDelay()
*/
uint16_t MtDevice::rs485TransmissionDelay() const
{
	XsMessage snd(XMID_ReqTransmitDelay), rcv;
	if (!doTransaction(snd, rcv))
		return 0;

	return rcv.getDataShort();
}

/*! \brief Set the RS485 acknowledge transmission delay of the device
	\param delay The delay value
	\returns True if successful
	\see rs485TransmissionDelay()
*/
bool MtDevice::setRs485TransmissionDelay(uint16_t delay)
{
	XsMessage snd(XMID_SetTransmitDelay, 2);
	snd.setBusId((uint8_t)busId());
	snd.setDataShort(delay);

	return doTransaction(snd);
}

/*! \brief Request data from the motion tracker
	\details The reply is handled by the live data path.
	This functionality is only available if outputSkipFactor() is set to 0xffff.
	\returns True if successful
*/
bool MtDevice::requestData()
{
	XsMessage snd(XMID_ReqData);
	snd.setBusId((uint8_t)busId());

	return sendRawMessage(snd);
}

/*! \brief Run a self test
	\returns The self test result
*/
XsSelfTestResult MtDevice::runSelfTest()
{
	XsMessage snd(XMID_RunSelfTest, 0);
	snd.setBusId((uint8_t)busId());
	XsMessage rcv;
	if (!doTransaction(snd, rcv, 3000))
		return XsSelfTestResult();

	return XsSelfTestResult::create(rcv.getDataShort());
}

/*! \copybrief XsDevice::storeFilterState
*/
bool MtDevice::storeFilterState()
{
	if (deviceState() == XDS_Config)
	{
		XsMessage snd(XMID_StoreFilterState);
		snd.setBusId((uint8_t)busId());

		if (doTransaction(snd))
			return true;
	}
	return false;
}

/*! \brief Calculates the frequency
	\param baseFrequency The base frequency to calculate with
	\param skipFactor The skip factor to calculate with
	\returns The calculated frequency
*/
int MtDevice::calcFrequency(int baseFrequency, uint16_t skipFactor)
{
	int result = baseFrequency / (skipFactor + 1);
	return result;
}

///@{ \name Log files
/*!
	\brief Set the read position of the open log file to the start of the file.

	If software filtering is enabled, the appropriate filters will be restarted
	as if the file was just opened. This function sets the lastResult value
	to indicate success (XRV_OK) or failure.

	\return True if successful
	\sa lastResult()
	\note This is a low-level file operation.
	\internal
*/
bool MtDevice::resetLogFileReadPosition()
{
	JLDEBUGG("");

	if (!XsDevice::resetLogFileReadPosition())
		return false;

	return true;
}

///@} end Log files

uint32_t MtDevice::supportedStatusFlags() const
{
	// essentially an unknown device, assume everything is supported
	return ~(uint32_t)0;
}
