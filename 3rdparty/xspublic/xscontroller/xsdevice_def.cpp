
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

#include "xsdevice_def.h"
#include <xstypes/xsdatapacket.h>
#include <xscommon/xsens_threadpool.h>
#include <xstypes/xsdeviceid.h>
#include <xstypes/xsstring.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xscanoutputconfigurationarray.h>
#include <xstypes/xsdeviceidarray.h>
#include <xstypes/xsintarray.h>
#include <xstypes/xsportinfo.h>
#include <xstypes/xssyncsettingarray.h>
#include "protocolhandler.h"
#include "communicator.h"
#include "mtbdatalogger.h"
#include <xstypes/xsbaud.h>
#include <xstypes/xsfilterprofile.h>
#include "xsselftestresult.h"
#include <xstypes/xsscrdata.h>
#include <xstypes/xscalibrateddata.h>
#include <xstypes/xsresetmethod.h>
#include <xstypes/xssyncsetting.h>
#include "nmea_protocolhandler.h"
#include <xstypes/xsrssi.h>
#include "supportedsyncsettings.h"
#include "messageserializer.h"
#include "xsdeviceptrarray.h"
#include <functional>
#include <xstypes/xsdatapacketptrarray.h>
#include <xstypes/xsfilterprofilearray.h>
#include <xstypes/xsstringoutputtypearray.h>
#include "xsdef.h"
#include "xsiccrepmotionresult.h"

//! \cond DOXYGEN_SHOULD_SKIP_THIS
using namespace xsens;
using namespace XsTime;

#if 1 && defined(XSENS_RELEASE) && defined(XSENS_DEBUG)
	// prevent spamming logs with identical "packet missed" loglines
	#define ONLYFIRSTMTX2	if (!deviceId().isMtx2() || firstChild() == this)
#else
	#define ONLYFIRSTMTX2
#endif
//! \endcond

#define TOADUMP	0	// set to 0 to disable
#if TOADUMP
#define CREATETOADUMPFILE()	if (this == master()) do { if (m_toaDumpFile != nullptr) { fflush(m_toaDumpFile); fclose(m_toaDumpFile); } m_toaDumpFile = fopen(xprintf("toadump_%08X_%p_%llu.csv", deviceId().toInt(), this, XsTimeStamp::nowMs()).c_str(), "wt"); } while(0)
#else
#define CREATETOADUMPFILE()	((void)0)
#endif

/*! \fn XsDevice::clearCallbackHandlers(bool chain = true)
	\brief Clear the callback handler list
	\param chain Whether to clear the callback handlers of all child devices as well (true, default)
	or just the callback handlers of this %XsDevice object (false)
*/

/*! \fn XsDevice::addCallbackHandler(XsCallbackPlainC* cb, bool chain = true)
	\brief Add a callback handler to the list
	\param cb The handler to add to the list.
	\param chain When set to true (default) the callback is added to child devices as well
	\note NULL and duplicate handlers are ignored, but chaining is still done.
*/

/*! \fn XsDevice::removeCallbackHandler(XsCallbackPlainC* cb, bool chain = true)
	\brief Remove a handler from the list
	\param cb The handler to remove from the list.
	\param chain When set to true (default) the callback is added to child devices as well
	\note If \a cb is not found in the list or if \a cb is NULL, the list is not changed, but
			chaining is still done.
*/

/*!	\brief Construct an empty device with device id \a id
	\param id The device ID to construct with
*/
XsDevice::XsDevice(XsDeviceId id)
	: m_latestLivePacket(new XsDataPacket)
	, m_latestBufferedPacket(new XsDataPacket)
	, m_unavailableDataBoundary(-1)
	, m_deviceId(id)
	, m_state(XDS_Initial)
	, m_connectivity(XCS_Disconnected)
	, m_communicator(nullptr)
	, m_logFileInterface(0)
	, m_master(this)
	, m_refCounter(0)
	, m_writeToFile(false)
	, m_isInitialized(false)
	, m_terminationPrepared(false)
	, m_gotoConfigOnClose(true)
	, m_justWriteSetting(false)
	, m_skipEmtsReadOnInit(true)
	, m_options(XSO_None)
	, m_startRecordingPacketId(-1)
	, m_stopRecordingPacketId(-1)
	, m_stoppedRecordingPacketId(-1)
	, m_lastAvailableLiveDataCache(new XsDataPacket)
	, m_toaDumpFile(nullptr)
{
	CREATETOADUMPFILE();
	JLDEBUGG("Created device " << deviceId());
}

/*!	\brief Construct a device using \a inf for communication
	\details Using this constructor implies that this device is a master device (master() returns this).
	\param comm The Communicator to use for this device
*/
XsDevice::XsDevice(Communicator* comm)
	: m_latestLivePacket(new XsDataPacket)
	, m_latestBufferedPacket(new XsDataPacket)
	, m_unavailableDataBoundary(-1)
	, m_deviceId(0)
	, m_state(XDS_Initial)
	, m_connectivity(XCS_Disconnected)
	, m_communicator(comm)
	, m_logFileInterface(0)
	, m_master(this)
	, m_refCounter(0)
	, m_writeToFile(false)
	, m_isInitialized(false)
	, m_terminationPrepared(false)
	, m_gotoConfigOnClose(true)
	, m_justWriteSetting(false)
	, m_skipEmtsReadOnInit(false)
	, m_options(XSO_None)
	, m_startRecordingPacketId(-1)
	, m_stopRecordingPacketId(-1)
	, m_stoppedRecordingPacketId(-1)
	, m_lastAvailableLiveDataCache(new XsDataPacket)
	, m_toaDumpFile(nullptr)
{
	// put callback managers and callbacks in place
	copyCallbackHandlersFrom(m_communicator);
	addChainedManager(m_communicator);

	m_deviceId = m_communicator->masterDeviceId();

	m_communicator->setMasterDevice(this);

	if (m_communicator->isPortOpen())
		m_connectivity = XCS_PluggedIn;
	else if (m_communicator->isReadingFromFile())
	{
		m_state = XDS_Measurement;
		m_connectivity = XCS_File;
	}
	CREATETOADUMPFILE();
	JLDEBUGG("Created device " << deviceId());
}

/*!	\brief Construct a device with device id \a childDeviceId for master \a masterDevice
	\param masterDevice The master device ID to construct for
	\param childDeviceId The child device ID to construct with
	\details Communication uses \a masterDevice's channel
*/
XsDevice::XsDevice(MtContainer *masterDevice, const XsDeviceId &childDeviceId)
	: m_latestLivePacket(new XsDataPacket)
	, m_latestBufferedPacket(new XsDataPacket)
	, m_unavailableDataBoundary(-1)
	, m_deviceId(childDeviceId)
	, m_state(XDS_Initial)
	, m_logFileInterface(0)
	, m_refCounter(0)
	, m_writeToFile(false)
	, m_isInitialized(false)
	, m_terminationPrepared(false)
	, m_gotoConfigOnClose(true)
	, m_justWriteSetting(false)
	, m_skipEmtsReadOnInit(true)
	, m_options(XSO_None)
	, m_startRecordingPacketId(-1)
	, m_stopRecordingPacketId(-1)
	, m_stoppedRecordingPacketId(-1)
	, m_lastAvailableLiveDataCache(new XsDataPacket)
	, m_toaDumpFile(nullptr)
{
	(void)masterDevice;
}

/*! \brief Destroy the device */
XsDevice::~XsDevice()
{
	JLDEBUGG(this << " did " << deviceId() << " refcounter " << m_refCounter.load());
	LockSuspendable locky(&m_deviceMutex, LS_Write);	// This is to make sure that we're not deleting the object while some other thread is using it. Especially the communicator threads tend to still be active at this point.

	assert(m_refCounter.load() == 0);
	assert(m_terminationPrepared);

	try {
		clearProcessors();
		clearExternalPacketCaches();

		if (m_latestLivePacket)
		{
			delete m_latestLivePacket;
			m_latestLivePacket = nullptr;
		}
		if (m_latestBufferedPacket)
		{
			delete m_latestBufferedPacket;
			m_latestBufferedPacket = nullptr;
		}
		if (m_lastAvailableLiveDataCache)
		{
			delete m_lastAvailableLiveDataCache;
			m_lastAvailableLiveDataCache = nullptr;
		}

		locky.unlock();	// the lock will interfere with thread termination
		if (isMasterDevice() && m_communicator)
		{
			removeChainedManager(m_communicator);
			m_communicator->destroy();
		}

		JLDEBUGG("object: " << this << " done");

		if (m_toaDumpFile)
		{
			fflush(m_toaDumpFile);
			fclose(m_toaDumpFile);
			m_toaDumpFile = nullptr;
		}
	}
	catch(...)
	{
	}
}

/*! \brief Return the master device of this device
	\details This function returns the master device of the current device. This may be the device
	itself
	\returns The master device of the device
*/
XsDevice *XsDevice::master() const
{
	return m_master;
}

/*! \cond XS_INTERNAL */
void XsDevice::clearProcessors()
{
}

/*! \brief Stop the processing thread before beginning to destroy the object
	\details This should be called at the start of every inheriting destructor
*/
void XsDevice::prepareForTermination()
{
	if (!m_terminationPrepared)
	{
		JLDEBUGG("Preparing " << deviceId() << " for termination");

		updateDeviceState(XDS_Destructing);

		if (isMasterDevice())
		{
			if (m_gotoConfigOnClose)
				gotoConfig();
			if (m_communicator != nullptr)
				m_communicator->closePort();
		}
		// finishLastProcessingTask();	// make sure no processing is going on in the background either...
		m_terminationPrepared = true;
	}
}

/*! \brief Update the device state immediately.
	\details The function marks the completion of a device state change. The callbacks that mark the end
	of a state transition should only be called from within this function or its overrides.
	\param newState The desired state
	\note The function can be overridden because some devices support different states
*/
void XsDevice::updateDeviceState(XsDeviceState newState)
{
	LockSuspendable locky(&m_deviceMutex, LS_Write);
	LockGuarded lockG(&m_deviceMutex);

	// we don't allow any transitions out of the (final) destructing state
	if (m_state == XDS_Destructing)
		return;

	XsDeviceState oldState = m_state;
	if (oldState != newState)
	{
		ONLYFIRSTMTX2
		JLDEBUGG("did: " << m_deviceId << " new: " << newState << " old: " << m_state);
		// some special case handling
		switch (newState)
		{
		case XDS_FlushingData:	// to
			switch (oldState)
			{
			case XDS_Measurement:	// from
				return;

			case XDS_Recording:	// from
				if (m_stopRecordingPacketId == -1 && isMasterDevice())
					m_stopRecordingPacketId = latestLivePacketId();
				m_stoppedRecordingPacketId = m_stopRecordingPacketId;
				ONLYFIRSTMTX2
				JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
				break;

			case XDS_WaitingForRecordingStart:	// from
				updateDeviceState(XDS_Measurement);
				return;

			default:
				break;
			}
			break;

		case XDS_Recording:	// to
			switch (oldState)
			{
			case XDS_Measurement:	// from
				m_stopRecordingPacketId = -1;
				m_stoppedRecordingPacketId = -1;
				if (m_startRecordingPacketId == -1 && isMasterDevice() && latestLivePacketId() >= 0)
					m_startRecordingPacketId = latestLivePacketId()+1;
				ONLYFIRSTMTX2
				JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
				break;
			default:
				break;
			}
			break;

		case XDS_WaitingForRecordingStart:	// to
			switch (oldState)
			{
			case XDS_Measurement:	// from
				m_stopRecordingPacketId = -1;
				m_startRecordingPacketId = -1;
				m_stoppedRecordingPacketId = -1;
				ONLYFIRSTMTX2
				JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
				break;
			default:
				break;
			}
			break;

		case XDS_Measurement:	// to
			switch (oldState)
			{
			case XDS_Recording:		// from
			case XDS_FlushingData:	// from
				m_stoppedRecordingPacketId = m_stopRecordingPacketId;
				if (m_stoppedRecordingPacketId == -1 && isMasterDevice())
					m_stoppedRecordingPacketId = latestLivePacketId();
				m_stopRecordingPacketId = -1;
				m_startRecordingPacketId = -1;
				//m_latestBufferedPacket->clear();
				ONLYFIRSTMTX2
				JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
				break;

			case XDS_Config:
			default:
				m_stopRecordingPacketId = -1;
				m_startRecordingPacketId = -1;
				m_stoppedRecordingPacketId = -1;
				ONLYFIRSTMTX2
				JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
				resetPacketStamping();
				reinitializeProcessors();
				break;
			}
			break;

		default:
			break;
		}

		m_state = newState;
		locky.unlock();
		onDeviceStateChanged(this, newState, oldState);
	}
}
/*! \endcond */

/*! \brief Find the child device with \a deviceid
	\details This function returns the child device of the current device that matches the given ID.
	\param deviceid The device ID to search for
	\returns A pointer to the found %XsDevice or 0 if the device could not be found
*/
XsDevice *XsDevice::findDevice(XsDeviceId deviceid)
{
	if (deviceid == m_deviceId)
		return this;
	return nullptr;
}

/*! \brief Find the child device with \a deviceid
	\details This function returns the child device of the current device that matches the given ID.
	\param deviceid The device ID to search for
	\returns A pointer to the found %XsDevice or 0 if the device could not be found
*/
XsDevice const *XsDevice::findDeviceConst(XsDeviceId deviceid) const
{
	return const_cast<XsDevice const*>(const_cast<XsDevice*>(this)->findDevice(deviceid));
}

/*! \brief On closePort the device will go to config by default, with this function it is possible to prevent that.
	\param gotoConfigOnClose boolean */
void XsDevice::setGotoConfigOnClose(bool gotoConfigOnClose)
{
	m_gotoConfigOnClose = gotoConfigOnClose;
}
/*! \brief Get the batterylevel of this device
	The battery level is a value between 0 and 100 that indicates the remaining capacity as a percentage.
	Due to battery characteristics, this is not directly the remaining time, but just a rough indication.

	Bodypack: The amount of time remaining for measurement given any battery level greatly depends on the type of batteries used,
				the number of sensors attached to the Bodypack and the used output options.
	Mtw: The last known battery level for this motion tracker. First call \sa requestBatteryLevel to have a battery level available.
		 The callback \sa onInfoResponse with ID XIR_BatteryLevel will indicate when the requested battery level is available.
		 This function is available in both config and measurement mode.
		 For devices in wired mode this function can be called without calling \sa requestBatteryLevel first
	\returns The battery level in the range 0-100
*/
int XsDevice::batteryLevel() const
{
	return 0;
}

/*! \brief Get the legacy update rate of the device
	\details This function is only valid for devices in legacy mode.
	\returns The legacy update rate of the device
 */
int XsDevice::updateRate() const
{
	return 0;
}

/*! \brief Set the legacy update rate of the device
	\param[in] rate The desired legacy update rate for the device
	\returns true if the update rate was successfully set
	\sa deviceMode \sa setDeviceMode
*/
bool XsDevice::setUpdateRate(int)
{
	return false;
}

/*! \brief Returns the device option flags
	\returns The current configured device option flags
*/
XsDeviceOptionFlag XsDevice::deviceOptionFlags() const
{
	return XDOF_None;
}

/*! \cond XS_INTERNAL */
/*! \brief Find the output configuration for \a dataType
 */
XsOutputConfiguration XsDevice::findConfiguration(XsDataIdentifier dataType) const
{
	XsDataIdentifier mask;
	if ((dataType & XDI_TypeMask) == dataType)
		mask = XDI_TypeMask;
	else
		mask = XDI_FullTypeMask;

	XsOutputConfigurationArray cfg = outputConfiguration();
	auto item = std::find_if(cfg.begin(), cfg.end(),
		[&](const XsOutputConfiguration &cfg)
		{
			return (cfg.m_dataIdentifier & mask) == dataType;
		}

	);
	if (item == cfg.end())
		return XsOutputConfiguration();
	return *item;
}
/*! \endcond */

/*! \brief Returns the currently configured update rate for the supplied \a dataType
	\details This function checks if the configured output contains \a dataType and returns the
	associated update rate. In some cases 65535 (0xFFFF) will be returned, which means 'as fast
	as possible'. This applies to things like a packet counter, which is sent with every packet
	and can therefore have an unpredictable update rate.
	This function only checks the output configured in the device, not possible computed data
	\param dataType The type of data to get the update rate for.
	\returns The requested update rate or 0 if the type is not configured for output
*/
int XsDevice::updateRateForDataIdentifier(XsDataIdentifier dataType) const
{
	XsOutputConfiguration cfg = findConfiguration(dataType);

	if (cfg.m_dataIdentifier == XDI_None)
		return updateRate();

	return cfg.m_frequency;
}

/*! \brief Returns the currently configured update rate for the supplied \a dataType
	\details This function checks if the configured output contains \a dataType and returns the
	associated update rate. In some cases 65535 (0xFFFF) will be returned, which means 'as fast
	as possible'. This applies to things like a packet counter, which is sent with every packet
	and can therefore have an unpredictable update rate.
	Where updateRateForDataIdentifier only checks the outputs configured in the device, this function also
	checks what can and will be computed from the data.
	\see updateRateForDataIdentifier
	\param dataType The type of data to get the update rate for.
	\returns The requested update rate or 0 if the type is not configured for output
*/
int XsDevice::updateRateForProcessedDataIdentifier(XsDataIdentifier dataType) const
{
	return updateRateForDataIdentifier(dataType);
}

/*! \brief Returns if the currently configured output contains \a dataType
	\param dataType The type of data to check the output for.
	\returns true if \a dataType is configured for output
	\sa outputConfiguration \sa hasProcessedDataEnabled \sa updateRateForDataIdentifier
*/
bool XsDevice::hasDataEnabled(XsDataIdentifier dataType) const
{
	return checkDataEnabled(dataType, outputConfiguration());
}

/*! \cond XS_INTERNAL */
/*! \brief Returns true if the supplied \a configurations contains \a dataType */
bool XsDevice::checkDataEnabled(XsDataIdentifier dataType, XsOutputConfigurationArray const & configurations)
{
	XsDataIdentifier mask;
	if ((dataType & XDI_TypeMask) == dataType)
		mask = XDI_TypeMask;
	else // if ((dataType & XDI_FullTypeMask) == dataType)
		mask = XDI_FullTypeMask;

	dataType = dataType & mask;

	for (XsOutputConfigurationArray::const_iterator i = configurations.begin(); i != configurations.end(); ++i)
	{
		if (dataType == (i->m_dataIdentifier & mask))
			return true;
	}
	return false;
}
/*! \endcond */

/*! \brief Returns if the currently configured output contains \a dataType after processing on the host
	\details Where hasDataEnabled() only checks the outputs configured in the device, this function also
	checks what can and will be computed from the data.
	\param dataType The type of data to check the output for.
	\returns true if \a dataType is configured for output
	\sa hasDataEnabled \sa processedOutputConfiguration \sa updateRateForDataIdentifier
*/
bool XsDevice::hasProcessedDataEnabled(XsDataIdentifier ) const
{
	return false;
}

/*! \brief Return the firmware version
	\returns The firmware version of the live device
	\note The firmware version is not stored in mtb files, so when reading from file this function will
	return an empty %XsVersion object
*/
XsVersion XsDevice::firmwareVersion() const
{
	return m_firmwareVersion;
}

/*! \cond XS_INTERNAL */
/*! \brief Set the cached firmware version of the device
*/
void XsDevice::setFirmwareVersion(const XsVersion &version)
{
	if (m_firmwareVersion != version)
#ifdef XSENS_DEBUG
		JLDEBUGG("Device " << deviceId() << " has firmware version " << version.toString());
#else
		JLWRITEG("Device " << deviceId() << " has firmware version " << version.toString());
#endif
	m_firmwareVersion = version;
}

/*!	\brief Extracts the firmware version of the device.
 *	\param message: XMID_FirmwareRevision message.
 */
void XsDevice::extractFirmwareVersion(XsMessage const& message)
{
	XsVersion old = m_firmwareVersion;
	m_firmwareVersion = XsVersion(message.getDataByte(0), message.getDataByte(1), message.getDataByte(2));
	if (message.getDataSize() > 3)
		m_firmwareVersion.setBuild(message.getDataLong(3));
	if (message.getDataSize() > 7)
		m_firmwareVersion.setReposVersion(message.getDataLong(7));
	if (old != m_firmwareVersion)
#ifdef XSENS_DEBUG
		JLDEBUGG("Device " << deviceId() << " has firmware version " << m_firmwareVersion.toString());
#else
		JLWRITEG("Device " << deviceId() << " has firmware version " << m_firmwareVersion.toString());
#endif
}

/*! \brief Return the related Communicator */
Communicator *XsDevice::communicator() const
{
	return m_communicator;
}

/*! \brief Return the related Communicator for logging */
DataLogger *XsDevice::logFileInterface() const
{
	return m_logFileInterface;
}

/*! \brief Set the device id of this device
*/
void XsDevice::setDeviceId(const XsDeviceId &deviceid)
{
	m_deviceId = deviceid;
}
/*! \endcond */

/*! \brief Return the device ID of the device
	\details Each Xsens device has a unique ID. The ID identifies the device as well as the
	product family it belongs to.
	\returns The device ID
*/
XsDeviceId XsDevice::deviceId() const
{
	return m_deviceId;
}

/*! \brief returns whether this device is in a master role regarding the device synchronization
	\returns true if the device has a synchronization master role
*/
bool XsDevice::isSyncMaster() const
{
	return false;
}

/*! \brief returns whether this device is in a slave role regarding the device synchronization
	\returns true if the device has a synchronization slave role
*/
bool XsDevice::isSyncSlave() const
{
	return false;
}

/*! \brief Return the device with bus ID \a busid
	\param busid The busid to serach for
	\returns The XsDevice corresponding to the supplied \a busid
*/
XsDevice *XsDevice::deviceAtBusId(int busid)
{
	if (isMasterDevice() && (busid == XS_BID_MASTER || busid == 1))
		return this;
	return nullptr;
}

/*! \brief Return the device with bus ID \a busid
	\param busid The busid to serach for
	\returns The const XsDevice corresponding to the supplied \a busid
*/
const XsDevice *XsDevice::deviceAtBusIdConst(int busid) const
{
	// prevent code duplication, go through the non-const implementation
	return const_cast<XsDevice const*>(const_cast<XsDevice*>(this)->deviceAtBusId(busid));
}

/*! \brief Restart the software filter used by this device */
void XsDevice::restartFilter()
{
}

/*!	\brief Get the result value of the last operation.
	\details The result values are codes that describe a failure in more detail.
	\returns the last known error code
	\sa resultText(XsResultValue), lastResultText()
*/
XsResultValue XsDevice::lastResult() const
{
	return m_lastResult;
}

/*!	\brief Get the accompanying error text for the value returned by lastResult()
	It may provide situation-specific information instead.
	\returns a human readable error description
	\sa resultText(XsResultValue), lastResult()
*/
XsString XsDevice::lastResultText() const
{
	return m_lastResult.lastResultText();
}

/*! \brief Returns true if this is a motion tracker
	\returns true if this is a motion tracker or false if it is a master device such as an Awinda
	Station or a Bodypack
*/
bool XsDevice::isMotionTracker() const
{
	return false;
}

/*! \brief Returns the length of the data in the legacy MTData packets that the device will send in
	measurement mode.
	\details This function will only return a value when the device is configured for legacy output,
	otherwise it will return 0.
	\returns The data size of the MTData packets that will be sent by the device
	\sa setDeviceMode \sa deviceMode
*/
int XsDevice::dataLength() const
{
	return 0;
}

/*! \brief The baud rate configured for cabled connection
	\details This differs from the baudRate() function in that it will return the configured value for
	a serial connection even if the device is currently not configured for serial communication (ie
	when it is connected with a direct USB cable or wirelessly), whereas the baudRate() function
	will return the baud rate of the current connection.
	\returns The configured baud rate
*/
XsBaudRate XsDevice::serialBaudRate() const
{
	return XBR_Invalid;
}

/*! \brief Get the baud rate (communication speed) of the serial port on which the given \a deviceId is connected.
	\details This differs from the serialBaudRate() function in that it will only return the baud rate
	of the current connection, whereas the serialBaudRate() function will return the configured value for
	a serial connection even if the device is currently not configured for serial communication (ie
	when it is connected with a direct USB cable or wirelessly).
	\returns The baud rate of the serial connection or XBR_Invalid
*/
XsBaudRate XsDevice::baudRate() const
{
	Communicator* comm = communicator();
	if (!comm)
		return XBR_Invalid;
	return comm->portInfo().baudrate();
}

/*! \brief The bus ID for this device
	\returns The bus ID of the device
*/
int XsDevice::busId() const
{
	return XS_BID_MASTER;
}

/*! \brief Change the serial baudrate to \a baudrate
	\details This function is only useful when using a serial communication channel, such as a
	serial-USB converter or a direct COM port. It is advised to make the baud rate as high as your
	platform allows, to minimize latency and problems with bandwidth.

	After setting the baudrate and communicating over the same communication channel, it is required to
	reset the device.
	\param baudrate The desired serial baudrate
	\returns true if the baud rate was successfully updated
*/
bool XsDevice::setSerialBaudRate(XsBaudRate baudrate)
{
	if (!isMasterDevice())
		return false;

	Communicator* comm = communicator();
	if (!comm)
		return false;

	if (comm->isReadingFromFile())
		return false;

	XsMessage snd(XMID_SetBaudrate, XS_LEN_BAUDRATE);
	snd.setBusId(XS_BID_MASTER);
	snd.setDataByte(XsBaud::rateToCode(baudrate));

	if (!doTransaction(snd, 500))
		return false;

	if (comm->portInfo().baudrate() == XBR_Invalid ||
		comm->portInfo().baudrate() == baudrate ||
		m_justWriteSetting)
		return true;

	if (!resetRemovesPort())
		return reset();
	else
		return true;
}

/*! \brief Get the current port configuration of a device
	\remarks Only Mti6x0 devices supported
	\returns The current port configurations of the device
	\sa setPortConfiguration, XsBaudCode
*/
XsIntArray XsDevice::portConfiguration() const
{
	return XsIntArray();
}

/*! \brief Change the port configuration of a device
	\details Configures the 2 ports of the 6x0 device
	The integers consist of:
	- bits 0:7 XsBaudcode
	- bit 8 Enable flow control
	- bit 9 Use 2nd stop bit
	- bit 10 Use Parity bit
	- bit 11 Even/odd parity
	- bit 12:15 reserved
	- bits 16:19 bits XsProtocol
	\param config An array of elements containing a configuration for each port (UART and RS232)
	\remarks Only Mti6x0 devices supported
	\returns true if the port configuration was successfully updated
	\sa portConfiguration, XsBaudCode
*/
bool XsDevice::setPortConfiguration(XsIntArray &config)
{
	(void)config;
	return false;
}

/*! \cond XS_INTERNAL */
/*! \brief Reset packet stamping by re-initializing the highestpacket
	\details Used after config-measurement cycling
*/
void XsDevice::resetPacketStamping()
{
	LockGuarded lockG(&m_deviceMutex);
	JLDEBUGG("did: " << deviceId());
	m_latestLivePacket->clear();
	m_latestBufferedPacket->clear();
	m_lastDataOkStamp = 0;
	m_unavailableDataBoundary = -1;
	m_packetStamper.resetTosEstimation();
	CREATETOADUMPFILE();
}
/*! \endcond */

/*! \brief Put this device in measurement mode
	\details Measurement mode is where the device is sampling data and producing inertial and orientation
	output.
	\returns true if the device was successfully put in measurement mode or was already in measurement
	mode
	\sa gotoConfig
*/
bool XsDevice::gotoMeasurement()
{
	JLDEBUGG(deviceId());

	if (!isMasterDevice())
	{
		m_lastResult.set(XRV_OTHER, deviceId().toString() << " is not a master device, can't switch to measurement mode");
		JLERRORG(m_lastResult.lastResultText());
		return false;
	}

	if (m_state == XDS_Measurement)	// if we're already in measurement mode, we can ignore this command
	{
		m_lastResult.set(XRV_OK, deviceId().toString() << " was already in measurement mode");
		JLDEBUGG(m_lastResult.lastResultText());
		return true;
	}

	Communicator* comm = communicator();
	if (!comm)
	{
		m_lastResult.set(XRV_OTHER, deviceId().toString() << " doesn't have a communicator, can't switch to measurement mode");
		JLERRORG(m_lastResult.lastResultText());
		return false;
	}

	if (comm->isReadingFromFile())
	{
		m_lastResult.set(XRV_OTHER, deviceId().toString() << " is reading from file, can't switch to measurement mode");
		JLERRORG(m_lastResult.lastResultText());
		return false;
	}

	XsResultValue res = comm->gotoMeasurement();
	if (res == XRV_OK)
	{
		// switch device (and children) to measuring state
		setDeviceState(XDS_Measurement);
		m_lastResult.set(XRV_OK, deviceId().toString() << " now in measurement mode");
		JLDEBUGG(m_lastResult.lastResultText());
		return true;
	}

	m_lastResult.set(XRV_OTHER, deviceId().toString() << " communicator refused with code " << res << " (" << XsResultValue_toString(res) << "), switching children back to config mode");
	JLERRORG(m_lastResult.lastResultText());
	return false;
}

/*! \brief Put the device in config mode
	\details Device settings can only be changed in config mode, since changing anything during
	measurement would mess up the sample timing.
	\returns true if the device was successfully put in config mode or was already in config mode
	\sa gotoMeasurement
*/
bool XsDevice::gotoConfig()
{
	JLDEBUGG(deviceId());
	if (!isMasterDevice())
	{
		m_lastResult.set(XRV_OTHER, deviceId().toString() << " is not a master device, can't switch to config mode");
		JLERRORG(m_lastResult.lastResultText());
		return false;
	}

	if (m_state == XDS_Config)	// if we're already in config mode, we can ignore this command
	{
		m_lastResult.set(XRV_OK, deviceId().toString() << " was already in config mode");
		JLDEBUGG(m_lastResult.lastResultText());
		return true;
	}

	Communicator* comm = communicator();
	if (!comm)
	{
		m_lastResult.set(XRV_OTHER, deviceId().toString() << " doesn't have a communicator, can't switch to config mode");
		JLERRORG(m_lastResult.lastResultText());
		return false;
	}

	if (comm->isReadingFromFile())
	{
		m_lastResult.set(XRV_INVALIDOPERATION, deviceId().toString() << " is reading from file, can't switch to config mode");
		JLDEBUGG(m_lastResult.lastResultText());
		return false;
	}

	XsResultValue res = comm->gotoConfig();
	if (res == XRV_OK)
	{
		setDeviceState(XDS_Config);
		m_lastResult.set(XRV_OK, deviceId().toString() << " now in config mode");
		JLDEBUGG(m_lastResult.lastResultText());
		return true;
	}

	m_lastResult.set(XRV_OTHER, deviceId().toString() << " communicator refused with code " << res << " (" << XsResultValue_toString(res) << "), can't switch to config mode");
	JLERRORG(m_lastResult.lastResultText());
	return false;
}

/*! \brief Return the state of this device
	\details The device state indiciates whether the device is in config mode, measuring, recording, etc
	\returns The state of the device
*/
XsDeviceState XsDevice::deviceState() const
{
	return m_state;
}

/*! \cond XS_INTERNAL */
/*! \brief Set the device state for this device (triggering callbacks) and relay it to all children
	\param state The device state to set
	\details To ensure proper switching, the switch is done threaded. To override functionality, use updateDeviceState()
	\returns The id of the task that marks the state change completion when \a waitForCompletion is false
*/
void XsDevice::setDeviceState(XsDeviceState state)
{
	// LockSuspendable locky(&m_deviceMutex, LS_Write);
	// lock is already acquired by updateDeviceState,
	updateDeviceState(state);
}
/*! \endcond */

/*! \brief Returns true if this is the master device (not a child of another device)
	\returns true if this is the master device
*/
bool XsDevice::isMasterDevice() const
{
	return this == master();
}

/*! \brief Returns true if this device can have child devices
	\returns true if this is a container device
*/
bool XsDevice::isContainerDevice() const
{
	return false;
}

/*! \brief Returns true if this is a standalone device (not a child of another device and not a container device)
	\returns true if this is a standalone device, equivalent to !isContainerDevice() && isMasterDevice()
*/
bool XsDevice::isStandaloneDevice() const
{
	return isMasterDevice() && !isContainerDevice();
}

/*! \brief Set the device option flags
	\param setFlags The option flags that must be set. Set to XDOF_None if no flags need to be set
	\param clearFlags The option flags that must be cleared. Set to XDOF_None if no flags need to be cleared
	\returns true if the device option flags were successfully altered
*/
bool XsDevice::setDeviceOptionFlags(XsDeviceOptionFlag setFlags, XsDeviceOptionFlag clearFlags)
{
	XsMessage snd(XMID_SetOptionFlags, 8);
	snd.setBusId(busId());
	MessageSerializer(snd) << (uint32_t)setFlags << (uint32_t)clearFlags;

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	return true;
}

/*! \brief Set the output configuration for this device
	\details When the function exits with a true value \a config will contain the actual configuration in the
	device after configuration. When it exits with false the contents of \a config are undefined.

	\note The \a config is updated to reflect frequency mismatches in desired configuration and actually possible
	configuration. As input, a frequency of 65535 (0xFFFF) may be supplied to indicate 'maximum output
	rate', but after configuration XDA will have put the actual maximum value in \a config.
	Similarly, some data types may not have a real update rate (ie. packet counter) and will return
	an update rate of 65535 (0xFFFF) when configured at any rate other than 0.

	\param config The desired output configuration
	\returns true if the output configuration was successfully updated
*/
bool XsDevice::setOutputConfiguration(XsOutputConfigurationArray& config)
{
	XsMessage snd(XMID_SetOutputConfiguration, 4);
	snd.setBusId(busId());
	bool wasEmpty = config.empty();
	MessageSerializer(snd) << config;

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return false;

	MessageDeserializer(rcv) >> config;
	if (wasEmpty && config.size() == 1 && config[0] == XsOutputConfiguration(XDI_None, 0))
		config.clear();
	return true;
}

/*! \brief Set the CAN output configuration for this device
	\details When the function exits with a true value \a config will contain the actual configuration in the
	device after configuration. When it exits with false the contents of \a config are undefined.

	\note The \a config is updated to reflect frequency mismatches in desired configuration and actually possible
	configuration. As input, a frequency of 65535 (0xFFFF) may be supplied to indicate 'maximum output
	rate', but after configuration XDA will have put the actual maximum value in \a config.
	Similarly, some data types may not have a real update rate (ie. packet counter) and will return
	an update rate of 65535 (0xFFFF) when configured at any rate other than 0.

	\param config The desired output configuration
	\returns true if the output configuration was successfully updated
*/
bool XsDevice::setCanOutputConfiguration(XsCanOutputConfigurationArray& config)
{
	(void)config;
	return false;
}

/*! \brief Set the CAN configuration for this device
   \param config Should consist of 8 bytes baudcode and 1 bit to enable CAN
   \returns true if the CAN output configuration was successfully updated
*/
bool XsDevice::setCanConfiguration(uint32_t config)
{
	(void)config;
	return false;
}

/*! \brief Return the full output configuration including post processing outputs
	\details This function return the list returned by outputConfiguration() and adds outputs
	that become available during post-processing.
	\returns The requested output configuration list
*/
XsOutputConfigurationArray XsDevice::processedOutputConfiguration() const
{
	return outputConfiguration();
}

/*! \brief Sets the string output mode for this device
	\param type The type to set
	\param period The period to set
	\param skipFactor The skipFactor to set
	\returns True if the device was successfully updated
*/
bool XsDevice::setStringOutputMode(uint16_t type, uint16_t period, uint16_t skipFactor)
{
	XsMessage sndType(XMID_SetStringOutputType);
	sndType.setBusId(XS_BID_MASTER);	// Always send to master device
	sndType.resizeData(2);
	sndType.setDataShort(type);

	if (!doTransaction(sndType))
		return false;

	if (type == 0)
		return true;

	XsMessage sndSkip(XMID_SetOutputSkipFactor);
	sndSkip.setBusId(busId());
	sndSkip.resizeData(2);
	sndSkip.setDataShort(skipFactor);

	if (!doTransaction(sndSkip))
		return false;

	m_config.masterInfo().m_outputSkipFactor = skipFactor;

	XsMessage sndPer(XMID_SetPeriod);
	sndPer.setBusId(XS_BID_MASTER);	// Always send to master device
	sndPer.resizeData(2);
	sndPer.setDataShort(period);

	if (!doTransaction(sndPer))
		return false;

	m_config.masterInfo().m_samplingPeriod = period;

	return true;
}

/*! \brief Ask the device for its supported string output types.
	\returns A list with the supported string output types.
*/
XsStringOutputTypeArray XsDevice::supportedStringOutputTypes() const
{
	return XsStringOutputTypeArray();
}

/*! \cond XS_INTERNAL */
/*! \brief Schedules the orientation reset
	\param method The reset method to use
	\returns True if successful
*/
bool XsDevice::scheduleOrientationReset(XsResetMethod method)
{
	if (method == XRM_StoreAlignmentMatrix && deviceState() != XDS_Config)
		return false;

	if (method != XRM_StoreAlignmentMatrix && deviceState() != XDS_Measurement && deviceState() != XDS_Recording)
		return false;

	XsMessage snd(XMID_ResetOrientation);
	snd.setBusId(busId());
	snd.resizeData(2);
	snd.setDataShort((uint16_t)method);

	if (!doTransaction(snd))
		return false;
	return true;
}
/*! \endcond */

/*!	\brief Send a custom message \a messageSend to the device and possibly wait for a result.
	\details If \a waitForResult is true, the function will wait for a result and put it in the given
	\a messageReceive. Otherwise the contents of messageReceive will not be altered.
	If an error message is received or the wait times out, \a messageReceive will contain an error
	message.

	\param messageSend The message to send to the device
	\param waitForResult true if it is required that the function waits for the appropriate reply. A
		valid reply always has a message ID that is one higher than the sent message ID.
	\param messageReceive When \a waitForResult is true, the reply will be put in this object.
	\param timeout Optional timeout in ms. When 0 is supplied (the default), the default timeout is used.
	\returns true if the message was successfully sent and when \a waitForResult is true the correct
	reply has been received
*/
bool XsDevice::sendCustomMessage(const XsMessage& messageSend, bool waitForResult, XsMessage& messageReceive, int timeout)
{
	return sendCustomMessage(messageSend, waitForResult, static_cast<XsXbusMessageId>(messageSend.getMessageId() + 1), messageReceive, timeout);
}

/*!	\brief Send a message directly to the communicator
	\param[in] message The message that will be sent
	\returns true if the message was successfully sent
*/
bool XsDevice::sendRawMessage(const XsMessage& message)
{
	Communicator* comm = communicator();
	return comm ? comm->writeMessage(message) : false;
}

/*! \cond XS_INTERNAL */
/*!	\brief Send a custom message \a messageSend to the device and possibly wait for a result.

	\details If \a waitForResult is true, the function will wait for a result and put it in the given
	\a messageReceive. Otherwise the contents of messageReceive will not be altered.
	If an error message is received or the wait times out, \a messageReceive will contain an error
	message.

	\param messageSend The message to send to the device
	\param waitForResult true if it is required that the function waits for the appropriate reply.
	A valid reply always has a message ID that is one higher than the sent message ID.
	\param messageId The message Id to wait for. Use this if the expected messageId is not the standard input messageid + 1
	\param messageReceive When \a waitForResult is true, the reply will be put in this object.
	\param timeout Optional timeout in ms. When 0 is supplied (the default), the default timeout is used.

	\returns true if the message was successfully sent and when \a waitForResult is true the correct
	reply has been received
*/
bool XsDevice::sendCustomMessage(const XsMessage& messageSend, bool waitForResult, XsXbusMessageId messageId, XsMessage& messageReceive, int timeout)
{
	Communicator* comm = communicator();
	if (!comm)
		return false;

	std::shared_ptr<ReplyObject> reply = comm->addReplyObject(messageId);
	if (!comm->writeMessage(messageSend))
		return false;

	if (waitForResult)
	{
		if (!timeout)
			timeout = comm->defaultTimeout();
		messageReceive = reply->message(timeout);
		if (messageReceive.getMessageId() != messageId)
			return false;
	}

	return true;
}

/*! \brief Let the communicator wait for for a message
	\param messageId The message Id to wait for.
	\param messageReceive The reply will be put in this object.
	\param timeout Optional timeout in ms. When 0 is supplied (the default), the default timeout is used.
	\returns true if the correct reply has been received
*/
bool XsDevice::waitForCustomMessage(XsXbusMessageId messageId, XsMessage &messageReceive, int timeout)
{
	Communicator* comm = communicator();
	if (!comm)
		return false;

	std::shared_ptr<ReplyObject> reply = comm->addReplyObject(messageId);

	if (!timeout)
		timeout = comm->defaultTimeout();
	messageReceive = reply->message(timeout);

	if (messageReceive.getMessageId() != messageId)
		return false;

	return true;
}

/*! \brief Let the communicator wait for for a message
	\param reply The reply object to wait for
	\param messageReceive The reply will be put in this object.
	\param timeout Optional timeout in ms. When 0 is supplied (the default), the default timeout is used.
	\returns true if the correct reply has been received
*/
bool XsDevice::waitForCustomMessage(std::shared_ptr<ReplyObject> reply, XsMessage &messageReceive, int timeout)
{
	if (!timeout && communicator())
		timeout = communicator()->defaultTimeout();
	messageReceive = reply->message(timeout);

	if (messageReceive.getMessageId() != reply->msgId())
		return false;

	return true;
}

/*!	\brief Add a reply object to the communicator to wait for a specific message
	\param messageId The message Id to wait for.
	\param data The data to check for in the first data byte location
	\returns A shared pointer to this object
*/
std::shared_ptr<ReplyObject> XsDevice::addReplyObject(XsXbusMessageId messageId, uint8_t data)
{
	Communicator* comm = communicator();
	if (!comm)
		return std::shared_ptr<ReplyObject>();

	return comm->addReplyObject(messageId, 0, 1, &data);
}

/*! \returns true if the supplied data message should be recorded to file
	\param msg The message to use
	\details This function should be overridden on a master device so we can check its device-specific messages.
	The function is only supplied XMID_Data and XMID_Data2 messages. If this needs to change, make sure to update
	the existing implementations accordingly.
*/
bool XsDevice::shouldDataMsgBeRecorded(const XsMessage &msg) const
{
	(void)msg;
	return (deviceState() == XDS_Recording);
}

/*! \brief Handle an XbusMessage
*/
void XsDevice::handleMessage(const XsMessage &msg)
{
	if (isMasterDevice())
		writeMessageToLogFile(msg);

	switch (msg.getMessageId())
	{
	case XMID_MtData2:
	{
		XsDataPacket packet(&msg);
		packet.setDeviceId(deviceId());
		handleDataPacket(packet);
	}	break;

	case XMID_Error:
		handleErrorMessage(msg);
		break;

	case XMID_Warning:
		handleWarningMessage(msg);
		break;

	case XMID_MasterIndication:
		handleMasterIndication(msg);
		break;

	default:
		JLDEBUGG("Handling non-data msg " << msg.getMessageId() << " bid " << JLHEXLOG((int)msg.getBusId()));
		handleNonDataMessage(msg);
		break;
	}
}

/*! \brief Process a message which is not a data message
*/
void XsDevice::handleNonDataMessage(const XsMessage &msg)
{
	onNonDataMessage(this, &msg);
}

/*! \brief Process an error message
*/
void XsDevice::handleErrorMessage(const XsMessage &msg)
{
	uint8_t errorCode = msg.getDataByte(0);
	XsResultValue xsResultValue = static_cast<XsResultValue>(errorCode);
	onNonDataMessage(this, &msg);
	onError(this, xsResultValue);
}

/*! \brief Process a warning message
*/
void XsDevice::handleWarningMessage(const XsMessage& msg)
{
	onNonDataMessage(this, &msg);
}

/*! \brief Inserts the packet ID and data packet into the data cache
	\param pid The packet ID to instert
	\param pack The data packet to insert
*/
void XsDevice::insertIntoDataCache(int64_t pid, XsDataPacket* pack)
{
	LockGuarded lockG(&m_deviceMutex);

	if (m_dataCache.empty())
		m_dataCache.insert(std::make_pair(pid, pack));
	else
	{
		auto it = m_dataCache.lower_bound(pid);
		if (it == m_dataCache.end())
			m_dataCache.insert(it, std::make_pair(pid, pack));
		else
		{
			if (it->first != pid)
			{
				m_dataCache.insert(it, std::make_pair(pid, pack));
			}
			else
			{
				it->second->merge(*pack, true);
				delete pack;
			}
		}
	}
}

/*! \brief Clears the data cache
*/
void XsDevice::clearDataCache()
{
	LockGuarded lockG(&m_deviceMutex);

	for (auto it : m_dataCache)
		delete it.second;
	m_dataCache.clear();
	//m_latestLivePacket->clear();
	m_latestBufferedPacket->clear();
	m_unavailableDataBoundary = -1;
}

/*! \brief Handles the inability to obtain specific measurement data
	\details Should be called when it is not possible to retransmit specific data. The data should be considered lost
	\param frameNumber The frame number of the data that is lost
*/
void XsDevice::handleUnavailableData(int64_t frameNumber)
{
	LockGuarded lockG(&m_deviceMutex);

	if (frameNumber < latestBufferedPacketId())
		return; // The recording stream already advanced itself beyond this frame. We don't need it
	if (latestLivePacketId() == -1)
		return;
	if (m_stopRecordingPacketId >= 0 && frameNumber > m_stopRecordingPacketId)
		return; // we're missing data past the end of the recording, ignore

	//JLDEBUGG("Device " << m_deviceId << " Updating m_unavailableDataBoundary from " << m_unavailableDataBoundary << " to " << std::max(m_unavailableDataBoundary, frameNumber));
	m_unavailableDataBoundary = (std::max)(m_unavailableDataBoundary, frameNumber);
	checkDataCache();
}

/*! \brief Returns true if it should do a recorded callback for a given \a data packet.
	\param p The reference to a data packet.
	\returns True if successful
*/
bool XsDevice::shouldDoRecordedCallback(XsDataPacket const& p) const
{
	if (p.empty())
		return false;

	if (isReadingFromFile())
		return true;

	switch (deviceState())
	{
	case XDS_Recording:
	case XDS_FlushingData:
		break;

	default:
		return false;
	}

	if (isStandaloneDevice())
		return true;

	if (m_stopRecordingPacketId >= 0 && p.packetId() > m_stopRecordingPacketId)
		return false;

	if (!p.containsFrameRange())
		return p.packetId() >= m_startRecordingPacketId;

	int64_t startPid = p.packetId() - p.frameRange().interval();
	return startPid >= m_startRecordingPacketId;
}

/*! \brief Handle an XbusDataPacket
	\param packet The data packet to handle
*/
void XsDevice::handleDataPacket(const XsDataPacket &packet)
{
	LockGuarded locky(&m_deviceMutex);
	if (m_terminationPrepared)
		return;	// we're being destroyed, abort handling of datapacket, state may be invalid

	m_lastDataOkStamp = XsTimeStamp::now();
	int64_t fastest = latestLivePacketConst().packetId();
	int64_t slowest = latestBufferedPacketConst().packetId();

	std::unique_ptr<XsDataPacket> pack(new XsDataPacket(packet));
	master()->m_packetStamper.stampPacket(*pack, latestLivePacket());	// always go through master for stamping packets so we have consistent timing
	int64_t current = pack->packetId();

#if TOADUMP
	fprintf(master()->m_toaDumpFile, "%llu,%llu,%llu\n", current, pack->timeOfArrival().msTime(), pack->estimatedTimeOfSampling().msTime());
#endif

#if 0
	ONLYFIRSTMTX2
	JLWRITEG(this << " [TOALOG] pid: " << current <<
			" did: " << deviceId() <<
			" awindaframenr: " << packet.awindaSnapshot().m_frameNumber <<
			" packetTOA: " << pack->timeOfArrival().msTime() <<
			" fastest: " << fastest <<
			" slowest: " << slowest <<
			" etos " << pack->estimatedTimeOfSampling().msTime());
	JLDEBUGG("stamped: " << current << " new latestlive: " << latestLivePacketConst().packetId());
#endif

	bool interpolate = false;
	if (fastest >= 0)
	{
		int64_t dpc = current - fastest;

		if (dpc > 1)
		{
			int64_t firstMissed = fastest + 1;
			int64_t lastMissed = current - 1;
			ONLYFIRSTMTX2
			JLDEBUGG("Detected " << (dpc-1) << " packets have been missed by device " << deviceId() << ", last was " << fastest << " (" << (uint16_t) fastest << ") current is " << current << " (" << (uint16_t) current << ")");
			onMissedPackets(this, (int) dpc-1, (int) firstMissed, (int) lastMissed);

			for (int64_t i = firstMissed; i <= lastMissed; ++i)
			{
				if (!expectingRetransmissionForPacket(i))
				{
					//JLDEBUGG("not expecting retransmission for packet " << i);
					handleUnavailableData(i);
					if (m_options & XSO_InterpolateMissingData)
						interpolate = true;
				}
			}
		}
	}
	else if (isReadingFromFile() && getStartRecordingPacketId() == -1)
		setStartRecordingPacketId(current);

	if (current >= fastest)
	{
		JLTRACEG("Processing (live) packet " << current);
		std::unique_ptr<XsDataPacket> copy(new XsDataPacket(*pack));
		processLivePacket(*copy);

		if (interpolate)
		{
			// when this returns true, the packet has been processed properly already so we should return
			if (interpolateMissingData(*copy, latestLivePacketConst(),
				[this](XsDataPacket* ppp)
				{
					handleDataPacket(*ppp);
					delete ppp;
				}))
			{
				return;
			}
		}

		// store result
		latestLivePacket().swap(*copy);

		// do callbacks
		if (!latestLivePacketConst().empty())
		{
			if (m_options & XSO_KeepLastLiveData)
				updateLastAvailableLiveDataCache(latestLivePacketConst());
			if (m_options & XSO_RetainLiveData)
				retainPacket(latestLivePacketConst());

			//if (isRecording())
			//	JLDEBUGG("Device " << m_deviceId << " triggering onLiveDataAvailable " << latestLivePacketConst().packetId());
			onLiveDataAvailable(this, &latestLivePacketConst());
			if (!isReadingFromFile())
				onDataAvailable(this, &latestLivePacketConst());
			if (isStandaloneDevice())
			{
				XsDevicePtrArray devs;
				const XsDataPacket* ppack = &latestLivePacketConst();
				devs.push_back(this);
				XsDataPacketPtrArray packs;
				packs.push_back(const_cast<XsDataPacket*>(ppack));
				onAllLiveDataAvailable(&devs, &packs);
				if (!isReadingFromFile())
					onAllDataAvailable(&devs, &packs);
			}
		}
	}
	else
	{
		// probably a retransmission
		//JLDEBUGG("Device " << deviceId() << " ignoring (live) packet " << current << " because it is older than " << fastest);
	}

	if (latestBufferedPacketConst().empty() || current > slowest)
	{
		// insert into cache
		//JLDEBUGG("Device " << deviceId() << " Adding (buffered) packet " << current);
		insertIntoDataCache(current, pack.get());
		pack.release();
		checkDataCache();
	}
	else
	{
		// not correct, weird old retransmission
		ONLYFIRSTMTX2
		JLDEBUGG("Device " << deviceId() << " ignoring (buffered) packet " << current << " because it is not newer than " << slowest);
	}
}
/*! \endcond */

/*! \brief Returns true if the device is currently in a measuring state
	\returns true if the device is currently in a measuring state
*/
bool XsDevice::isMeasuring() const
{
	switch (deviceState()) {
	case XDS_Measurement:
	case XDS_WaitingForRecordingStart:
	case XDS_Recording:
	case XDS_FlushingData:
		return true;

	default:
		return false;
	}
}

/*! \brief Returns true if the device is currently in a recording state
	\returns true if the device is currently in a recording state
*/
bool XsDevice::isRecording() const
{
	switch (deviceState()) {
	case XDS_WaitingForRecordingStart:
	case XDS_Recording:
	case XDS_FlushingData:
		return true;

	default:
		return false;
	}
}

/*! \brief Returns true if the device is reading from a file
	\returns true if the device is reading from a file
*/
bool XsDevice::isReadingFromFile() const
{
	if (!communicator())
		return false;
	assert(communicator());
	return communicator()->isReadingFromFile();
}

/*! \cond XS_INTERNAL */
/*! \brief Process a XsDataPacket (calibrate and filter) and put the results in the appropriate cache
*/
void XsDevice::processLivePacket(XsDataPacket &)
{
}

/*! \brief Process a XsDataPacket (calibrate and filter) and put the results in the appropriate cache
*/
void XsDevice::processBufferedPacket(XsDataPacket &)
{
}
/*! \endcond */

/*! \brief Clear the inbound data buffers of the device
*/
void XsDevice::flushInputBuffers()
{
	JLDEBUGG(this);
	if (isMasterDevice() && communicator())
		communicator()->flushPort();
	clearDataCache();
	resetPacketStamping();
}

/*! \brief Returns the synchronization role of the device
	\details For synchronization purposes a device can be a master, a slave, both or neither. This
	function returns the way the device is currently configured.
	\returns The synchronization role of the device
*/
XsSyncRole XsDevice::syncRole() const
{
	return XSR_None;
}

/*! \cond XS_INTERNAL */
/*! \brief Reads the device configuration
	\returns True if successful
*/
bool XsDevice::readDeviceConfiguration()
{
	JLDEBUGG("");
	if (isMasterDevice())
	{
		Communicator* comm = communicator();
		if (!comm)
			return false;

		if (comm->isReadingFromFile())
		{
			XsMessage rcv;

			rcv = comm->readMessageFromStartOfFile(XMID_Configuration, Communicator::configurationMessageSearchLimit());
			if (comm->lastResult() != XRV_OK)
				return false;
			m_config.readFromMessage(rcv);

			rcv = comm->readMessageFromStartOfFile(XMID_FirmwareRevision, Communicator::configurationMessageSearchLimit() * (busId() == XS_BID_MASTER ? 2 : (1+busId())));
			if (comm->lastResult() != XRV_OK)
				// We cannot determine the firmware version of the master, use the first sensor (fallback)
				m_firmwareVersion = XsVersion(m_config.deviceInfo(busId()).m_fwRevMajor, m_config.deviceInfo(busId()).m_fwRevMinor, m_config.deviceInfo(busId()).m_fwRevRevision);
			else
				extractFirmwareVersion(rcv);
		}
		else
		{
			if (!gotoConfig())
			{
				JLALERTG("Failed to go to config");
				return false;
			}

			XsMessage snd(XMID_Initbus);
			snd.setBusId(busId());

			if (!doTransaction(snd, 500))
			{
				JLALERTG("Failed to init bus");
				return false;
			}

			snd.setMessageId(XMID_ReqConfiguration);
			snd.setBusId((uint8_t)busId());

			XsMessage rcv;
			if (!doTransaction(snd, rcv, 5000))
			{
				JLALERTG("Failed to req configuration");
				return false;
			}

			m_config.readFromMessage(rcv);
			m_firmwareVersion = XsVersion();
			snd.setMessageId(XMID_ReqFirmwareRevision);
			snd.setBusId((uint8_t)busId());

			if (doTransaction(snd, rcv) && rcv.getMessageId() == XMID_FirmwareRevision)
				extractFirmwareVersion(rcv);
		}
		return true;
	}
	else
	{
		try
		{
			const XsMtDeviceConfiguration& devinf = deviceConfigurationConst().deviceInfo(deviceId());
			XsVersion old = m_firmwareVersion;
			m_firmwareVersion = XsVersion(devinf.m_fwRevMajor, devinf.m_fwRevMinor, devinf.m_fwRevRevision);
#ifndef XSENS_DEBUG
			// this is support debug info, we don't want to spam the debugger windows
			if (old != m_firmwareVersion)
				JLWRITEG("Device " << deviceId() << " has firmware version " << m_firmwareVersion.toString());
#endif
			return true;
		}
		catch (XsDeviceConfigurationException &)
		{
			return false;
		}
	}
	return false;
}

/*! \brief Returns a const reference to the device configuration
	\details The device configuration contains a summary of the devices connected to the same port.
	The function will always return the configuration for the port's main device.
	\returns A copy of the device configuration of the port
*/
XsDeviceConfiguration const& XsDevice::deviceConfigurationConst() const
{
	return m_config;
}

/*! \brief Returns a reference to the device configuration
	\details The device configuration contains a summary of the devices connected to the same port.
	The function will always return the configuration for the port's main device.
	\returns A reference to the device configuration of the port
*/
XsDeviceConfiguration& XsDevice::deviceConfigurationRef()
{
	return const_cast<XsDeviceConfiguration&>(deviceConfigurationConst());
}
/*! \endcond */

/*! \brief Returns the device configuration
	\details The device configuration contains a summary of the devices connected to the same port.
	The function will always return the configuration for the port's main device.
	\returns A copy of the device configuration of the port
*/
XsDeviceConfiguration XsDevice::deviceConfiguration() const
{
	return deviceConfigurationConst();
}

/*! \brief Restore the device to its factory default settings
	\returns true if the settings have been successfully restored
*/
bool XsDevice::restoreFactoryDefaults()
{
	if (deviceState() == XDS_Measurement || deviceState() == XDS_Recording)
		return false;

	XsMessage snd(XMID_RestoreFactoryDef);
	snd.setBusId(busId());

	if (!doTransaction(snd,2000))
		return false;

	return true;
}

/*! \brief Reset the device
	\details This function tells the device to reboot itself.
	\returns true if the device was successfully reset
*/
bool XsDevice::reset()
{
	return reset(false);
}

/*! \cond XS_INTERNAL */
/*! \brief Reset the device
	\details This function tells the device to reboot itself.
	\param skipDeviceIdCheck Set to true if the rescan should not verify the device id (not recommended)
	\returns true if the device was successfully reset
*/
bool XsDevice::reset(bool skipDeviceIdCheck)
{
	if (!isMasterDevice())
		return false;

	Communicator* comm = communicator();
	if (!comm)
		return false;

	if (!gotoConfig())
		return false;

	XsMessage snd(XMID_Reset);
	snd.setBusId(XS_BID_MASTER);

	std::shared_ptr<ReplyObject> wakeup = comm->addReplyObject(XMID_Wakeup);
	comm->writeMessage(snd);

	if (!reopenPort(false, skipDeviceIdCheck))
		return false;

	if (!gotoConfig())
		return false;

	return true;
}
/*! \endcond */

/*! \brief Reopens a port
	Uses rescan method to redetect a device. Also if USB descriptor has changed
	\param gotoConfig Set to true if the device should be put to config before port closure
	\param skipDeviceIdCheck Set to true if the rescan should not verify the device id (not recommended)
	\returns true if successful
*/
bool XsDevice::reopenPort(bool gotoConfig, bool skipDeviceIdCheck)
{
	setGotoConfigOnClose(gotoConfig);

	Communicator* comm = communicator();
	if (!comm)
		return false;

	return comm->reopenPort(OPS_OpenPort, skipDeviceIdCheck);
}

/*! \brief Write the emts/wms/xms of the device and all its children to the open logfile
*/
void XsDevice::writeDeviceSettingsToFile()
{
}

/*! \brief Create a log file for logging
	\param filename The desired path and filename of the log file
	\returns Result value indicating success (XRV_OK) or failure
*/
XsResultValue XsDevice::createLogFile(const XsString &filename)
{
	JLDEBUGG(filename);
	Communicator* comm = communicator();
	if (!comm || !comm->isPortOpen())
	{
		JLALERTG("No port open");
		return XRV_NOPORTOPEN;
	}
	if (logFileInterface())
	{
		JLERRORG("A file is already open");
		return XRV_ALREADYOPEN;
	}

	std::unique_ptr<MtbDataLogger> newfile(new MtbDataLogger);
	if (newfile->create(filename))
	{
		m_logFileInterface = newfile.release();
		JLDEBUGG("Creation ok");
		tm dateTime;
		getDateTime(&dateTime);
		getDateAsString((char*) m_config.masterInfo().m_date, &dateTime);
		getTimeAsString((char*) m_config.masterInfo().m_time, &dateTime);

		XsMessage msg;
		deviceConfiguration().writeToMessage(msg);
		m_logFileInterface->writeMessage(msg);
		writeDeviceSettingsToFile();
		return XRV_OK;
	}
	JLDEBUGG("Creation failed");
	newfile->close(true);
	removeChainedManager(m_logFileInterface);
	newfile.reset();
	return XRV_OUTPUTCANNOTBEOPENED;
}

/*! \brief Stores the current device configuration in a config file(.xsa)
	\param[in] filename The desired path and filename of the config file
	\returns Result value indicating success (XRV_OK) or failure
*/
XsResultValue XsDevice::createConfigFile(const XsString& )
{
	return XRV_NOTIMPLEMENTED;
}

/*! \brief Set an arbitrary alignment rotation quaternion.
	Use to rotate either L to the chosen frame L' or S to the chosen frame S'
	\param frame The frame to rotate
	\param quat The desired alignment rotation setting of the device.
	\returns true if the alignment rotation has been set successfully
	\see alignmentRotationQuaternion, setAlignmentRotationMatrix, alignmentRotationMatrix
*/
bool XsDevice::setAlignmentRotationQuaternion(XsAlignmentFrame frame, const XsQuaternion& quat)
{
	(void)frame;
	(void)quat;
	return false;
}

/*! \brief Retrieve the alignment rotation quaternion
	\param frame The frame of which to return the alignment rotation
	\returns The alignment rotation
	\see setAlignmentRotationQuaternion, setAlignmentRotationMatrix, alignmentRotationMatrix
*/
XsQuaternion XsDevice::alignmentRotationQuaternion(XsAlignmentFrame frame) const
{
	(void)frame;
	return XsQuaternion();
}

/*! \brief Set an arbitrary alignment rotation matrix
	Use to rotate either L to the chosen frame L' or S to the chosen frame S'
	\param frame The frame to rotate
	\param matrix The desired alignment rotation setting of the device. This should be an
	orthonormal 3x3 matrix.
	\returns true if the alignment rotation has been set successfully
	\see setAlignmentRotationQuaternion, alignmentRotationQuaternion, alignmentRotationMatrix
*/
bool XsDevice::setAlignmentRotationMatrix(XsAlignmentFrame frame, const XsMatrix& matrix)
{
	(void)frame;
	(void)matrix;
	return false;
}

/*! \brief Retrieve the alignment rotation matrix to rotate S to the chosen frame S'
	\param frame The frame of which to return the alignment rotation
	\returns The alignment rotation
	\see setAlignmentRotationQuaternion, alignmentRotationQuaternion, setAlignmentRotationMatrix
*/
XsMatrix XsDevice::alignmentRotationMatrix(XsAlignmentFrame frame) const
{
	(void)frame;
	return XsMatrix();
}

/*! \brief Loads a config file(.xsa) and configures the device accordingly
	\param[in] filename The desired path and filename of the config file
	\returns Result value indicating success (XRV_OK) or failure
*/
XsResultValue XsDevice::applyConfigFile(const XsString& )
{
	return XRV_NOTIMPLEMENTED;
}

/*! \brief Close the log file
	\returns true if the log file was successfully closed or never open
*/
bool XsDevice::closeLogFile()
{
	if (m_logFileInterface)
	{
		JLDEBUGG(m_logFileInterface);
		m_logFileInterface->close();
		removeChainedManager(m_logFileInterface);
		delete m_logFileInterface;
		m_logFileInterface = NULL;
	}
	return true;
}

/*!	\brief Start recording incoming data
	\details To record successfully, a log file should be created by calling createLogFile() before
	this function is called. startRecording(XsString, XsDeviceId) can be used to achieve the same result.
	\returns true if recording was successfully started
	\note Starting recording for a single non-main device will start a recording for the entire system.
	\sa createLogFile() \sa stopRecording()
*/
bool XsDevice::startRecording()
{
	JLDEBUGG("");
	if (!isMasterDevice())
		return false;

	if (deviceState() != XDS_Measurement)
		return false;

	writeFilterStateToFile();
	setDeviceState(XDS_Recording);

	return true;
}

/*! \brief Start recording incoming data through generating a virtual input trigger
	\note On devices without support for a start recording input trigger this function will default to XsDevice::startRecording
	\returns true if recording was successfully started
	\sa createLogFile() \sa stopRecording() \sa startRecording
*/
bool XsDevice::triggerStartRecording()
{
	return startRecording();
}

/*!	\brief Stop recording incoming data
	\returns true if recording was successfully stopped
	\note Stopping recording for a single non-main device will stop a recording for the entire system.
	\sa createLogFile() \sa startRecording()
*/
bool XsDevice::stopRecording()
{
	JLDEBUGG("");
	if (!isMasterDevice())
		return false;

	if (deviceState() != XDS_Recording)
		return false;

	setDeviceState(XDS_Measurement);
	return true;
}

/*! \cond XS_INTERNAL */
/*! \brief Returns whether the packet contains a retransmission or not */
bool XsDevice::packetContainsRetransmission(XsDataPacket const& packet)
{
	if (packet.containsAwindaSnapshot())
		return packet.isAwindaSnapshotARetransmission();

	return false;
}

/*! \brief Handles the end of the recording stream
*/
void XsDevice::endRecordingStream()
{
	LockGuarded lockG(&m_deviceMutex);

	if (m_dataCache.empty() || (m_startRecordingPacketId < 0))
		return;

	if (m_stopRecordingPacketId >= 0)
	{
		//The first packetId following the latestBufferedPacket (head of recording stream) is what is blocking possible cached data
		//If the recording stream is empty the beginning of the recording is unavailable (from start recording until the first packet in the cache)
		int64_t blockingPacketId = (latestBufferedPacketId() < 0) ? m_startRecordingPacketId : latestBufferedPacketId() + 1;

		while (blockingPacketId <= m_stopRecordingPacketId)
		{
			//Mark blocking packet as unavailable.
			handleUnavailableData(blockingPacketId);
			//If marking the blocking packet unavailable did not advance the head of the recording stream the next blocking packet follows the current one immediately
			blockingPacketId = (std::max)(blockingPacketId + 1, latestBufferedPacketId() + 1);
		}
	}

	checkDataCache();
	// clearDataCache();
}

/*! \brief Check if cache contains data to process */
void XsDevice::checkDataCache()
{
	LockGuarded lockG(&m_deviceMutex);
	// process available data
	while (!m_dataCache.empty())
	{
		auto it = m_dataCache.begin();
		//JLWRITEG("pid: " << it->second->packetId() << " range? " << it->second->containsFrameRange() << " retransmission? " << it->second->isAwindaSnapshotARetransmission() << " snapshotA,F? " << it->second->containsAwindaSnapshot() << "," << it->second->containsFullSnapshot());

		int64_t expectedPacketId = latestBufferedPacketId() < 0 ? -1 : latestBufferedPacketId() + 1;
		if (expectedPacketId < m_startRecordingPacketId)
		{
			//If there is a frame range (i.e. we get an interval) the packetId is equal to the end of that interval
			//The startRecordingPacketId always is equal to the start value of an interval. Therefore if using the startRecordingPacketId to
			//calculate the expected packetId for an ideal interval (no missing data) is to +1 the startRecordingPacketId
			//For an ideal (expected) situation this would be 1 higher than the startRecordingPacketId
			expectedPacketId = it->second->containsFrameRange() ? m_startRecordingPacketId + 1 : getStartRecordingPacketId();
			expectedPacketId = PacketStamper::calculateLargePacketCounter(expectedPacketId, latestLivePacketId(), PacketStamper::MTSCBOUNDARY);
		}
		int64_t packetId = it->first;

		auto missingDataIsUnavailable = [&](int64_t rFirst, int64_t rLast)
		{
			if ((m_stopRecordingPacketId >= 0 && rFirst > m_stopRecordingPacketId) ||
				(rLast-1 <= m_unavailableDataBoundary))
			{
				// notify unavailable data in recording states
				// when not recording, unavailable data is expected and should not be reported
				if (isRecording())
				{
					//JLDEBUGG("Device " << m_deviceId << " data unavailable in range " << rFirst << " - " << rLast);
					if (m_stopRecordingPacketId != -1 && rLast > m_stopRecordingPacketId)
						rLast = m_stopRecordingPacketId;
					if (rFirst < rLast)
					{
						ONLYFIRSTMTX2
						JLDEBUGG("Device " << m_deviceId << " Data is unavailable in reported range " << rFirst << " - " << rLast << " Stop Recording Packet ID " << m_stopRecordingPacketId << " Unavailable Data Boundary " << m_unavailableDataBoundary);
						for (int64_t r = rFirst; r < rLast; ++r)
							onDataUnavailable(this, r);
					}
					else
					{
						//ONLYFIRSTMTX2
						//JLDEBUGG("Device " << m_deviceId << " NOT Reporting data unavailable");
					}
				}

				return true;
			}
			return false;
		};

		int64_t rFirst = expectedPacketId >= 0 ? expectedPacketId : packetId;
		int64_t rLast = packetId;
		if (it->second->containsFrameRange())
		{
			XsRange rng = it->second->frameRange();
			rFirst = rng.first() + 1;
		}

		if ((expectedPacketId >= 0) && (packetId > expectedPacketId))
		{
			if (!missingDataIsUnavailable(rFirst, rLast))
			{
				//JLDEBUGG("Device " << m_deviceId << " Waiting for retransmissions in range " << rFirst << " - " << rLast);
				break;	// waiting for retransmissions
			}
		}
		// we need to 'else' here to avoid duplicate and erroneous missed packet handling
		else if (it->second->containsFrameRange())
		{
			if (rLast > rFirst)
			{
				if (!missingDataIsUnavailable(rFirst, rLast))
					break;	// waiting for retransmissions
			}
		}

		// do 'buffered' processing
		processBufferedPacket(*it->second);

		// store result
		latestBufferedPacket().swap(*it->second);
//		ONLYFIRSTMTX2
//		JLDEBUGG("latestBufferedPacket is now " << latestBufferedPacket().packetId() << " old: " << it->second->packetId());
		delete it->second;
		m_dataCache.erase(it);

		if (latestBufferedPacketConst().empty())
			continue;

		if (m_options & XSO_RetainBufferedData)
			retainPacket(latestBufferedPacketConst());

		// do callbacks
		const XsDataPacket& buf = latestBufferedPacketConst();
		//if (isRecording())
		//	JLDEBUGG("Device " << m_deviceId << " triggering onBufferedDataAvailable " << buf.packetId());
		onBufferedDataAvailable(this, &buf);
		bool doRecCallback = shouldDoRecordedCallback(buf);
		if (isReadingFromFile())
			onDataAvailable(this, &buf);
		if (doRecCallback)
			onRecordedDataAvailable(this, &buf);

		if (isStandaloneDevice())
		{
			XsDevicePtrArray devs;
			const XsDataPacket* ppack = &latestBufferedPacketConst();
			devs.push_back(this);
			XsDataPacketPtrArray packs;
			packs.push_back((XsDataPacket*)ppack);
			onAllBufferedDataAvailable(&devs, &packs);
			if (isReadingFromFile())
				onAllDataAvailable(&devs, &packs);
			if (doRecCallback)
				onAllRecordedDataAvailable(&devs, &packs);
		}
	}
}

/*! \brief Store a message in the log file if a log file is open */
void XsDevice::writeMessageToLogFile(const XsMessage &message)
{
	if (communicator()->isReadingFromFile())
		return;

	switch (message.getMessageId())
	{
	case XMID_MtData2:

	default:
		// always write all other messages to log file
		break;
	}

	if (!onWriteMessageToLogFile(this, &message))
		return;

	DataLogger* logInt = logFileInterface();
	if (!logInt)
		return;

	logInt->writeMessage(message);
}

/*! \brief Write filter states to file */
void XsDevice::writeFilterStateToFile()
{
}
/*! \endcond */

/*! \brief Ask the device for its supported update rates for the given \a dataType
	\param dataType The type of data to get the supported update rates for
	\returns A list with the supported update rates or an empty list in case of an error
*/
std::vector<int> XsDevice::supportedUpdateRates(XsDataIdentifier dataType) const
{
	(void)dataType;
	return std::vector<int>();
}

/*! \brief Returns true if the device has its BlueTooth radio enabled
	\returns true if the device has its BlueTooth radio enabled
	\sa setBlueToothEnabled
*/
bool XsDevice::isBlueToothEnabled() const
{
	return false;
}
/*! \brief Enable or disable the BlueTooth radio of the device
	\param enabled Set to true to enable the BlueTooth radio
	\returns true if the device was successfully updated
	\sa isBlueToothEnabled
*/
bool XsDevice::setBlueToothEnabled(bool enabled)
{
	(void)enabled;
	return false;
}

/*! \brief Returns if the Xbus is powering its child devices or not
	\details When the bus power is off, the child devices are disabled
	\returns true If the Xbus is currently providing power to its child devices
*/
bool XsDevice::isBusPowerEnabled() const
{
	return false;
}
/*! \brief Tell the Xbus to provide power to its child devices or not
	\details This function can be used to tell the Xbus to stop and start powering its child
	devices. By default when the Xbus starts up it will provide power to its child devices.
	Switching the power off can save a lot of energy, but powering the system up again will take some
	time, depending on the number of connected devices.
	\param enabled true to enable bus power, false to disable the bus power
	\returns true If the Xbus' bus power state was successfully updated.
*/
bool XsDevice::setBusPowerEnabled(bool enabled)
{
	(void)enabled;
	return false;
}
/*! \brief Tell the device to power down completely
	\details This function can be used to tell the device to shut down completely, requiring a physical
	button press on the device to power up again.
	\returns true if the device was successfully powered down
*/
bool XsDevice::powerDown()
{
	return false;
}
/*! \brief Returns the error mode of the device
	\details The error mode tells the device what to do if a problem occurs.
	\returns The currently configured error mode of the device
	\sa setErrorMode
*/
XsErrorMode XsDevice::errorMode() const
{
	return XEM_Invalid;
}
/*! \brief Sets the error mode of the device
	\details The error mode tells the device what to do if a problem occurs.
	\param errormode The desired error mode of the device
	\returns true if the device was successfully updated
	\sa errorMode
*/
bool XsDevice::setErrorMode(XsErrorMode errormode)
{
	(void)errormode;
	return false;
}

// MT Device functions

/*! \brief Set the 'heading offset' setting of the device
	\param offset The desired heading offset of the device in degrees
	\returns true if the device was successfully updated
	\sa headingOffset
*/
bool XsDevice::setHeadingOffset(double offset)
{
	(void)offset;
	return false;
}

/*! \brief Return the 'heading offset' setting of the device
	\returns The currently configured heading offset in degrees
*/
double XsDevice::headingOffset() const
{
	return 0;
}

/*! \brief Set the location ID of the device
	\details The location ID is a custom 16-bit ID that can be assigned to a device.
	\param id The desired location ID for the device
	\returns true if the Location ID was successfully updated
	\sa locationId
*/
bool XsDevice::setLocationId(int id)
{
	(void)id;
	return false;
}

/*! \brief Get the location ID of the device
	\details The location ID is a custom 16-bit ID that can be assigned to a device.
	\returns The current location ID stord in the device
	\sa setLocationId
*/
int XsDevice::locationId() const
{
	return 0;
}

/*! \brief Get the device given \a locId
 *
 * \param locId the location ID of the device we're looking for
 * \returns a pointer to the device if found, nullptr otherwise.
 */
XsDevice* XsDevice::getDeviceFromLocationId(uint16_t locId)
{
	if (locationId() == locId)
		return this;

	return nullptr;
}

/*!	\brief Returns the object alignment matrix of the device
	\returns The current 'object alignment matrix' setting of the device.
	\note This is legacy functionality to support backwards compatibility with older devices. For
	MT Mk4 devices it is suggested to use alignmentRotationQuaternion or alignmentRotationMatrix instead.
	\sa setObjectAlignmentMatrix() \sa headingOffset() \sa setHeadingOffset() \sa alignmentRotationQuaternion \sa alignmentRotationMatrix
*/
XsMatrix XsDevice::objectAlignment() const
{
	return XsMatrix();
}

/*!	\brief Sets the object alignment of the device to the given \a matrix.
	\param matrix The desired 'object alignment matrix' setting of the device. This should be an
	orthonormal 3x3 matrix.
	\returns true if the object alignment matrix was successfully written
	\note This is legacy functionality to support backwards compatibility with older devices. For
	MT Mk4 devices it is suggested to use setAlignmentRotationQuaternion or setAlignmentRotationMatrix instead.
	\sa objectAlignmentMatrix \sa headingOffset \sa setHeadingOffset \sa setAlignmentRotationQuaternion \sa setAlignmentRotationMatrix
*/
bool XsDevice::setObjectAlignment(const XsMatrix &matrix)
{
	(void)matrix;
	return false;
}

/*!	\brief Returns the 'Gravity Magnitude' of the device
	\details The Gravity Magnitude is the strength of the gravity where the measurements are done.
	Setting	this value precisely allows for more accurate measurements.
	\returns The current 'Gravity Magnitude' setting of the device.
	\sa setGravityMagnitude \sa setInitialPositionLLA \sa initialPositionLLA
*/
double XsDevice::gravityMagnitude() const
{
	return 0;
}

/*!	\brief Sets the 'Gravity Magnitude' of the device to the given value \a mag.
	\details The Gravity Magnitude is the strength of the gravity where the measurements are done.
	Setting	this value precisely allows for more accurate measurements.
	\param mag The desired 'Gravity Magnitude' setting of the device.
	\returns true if the Gravity Magnitude was successfully written
	\note The default value is usually computed from the last known Lat Lon Alt value
	\sa gravityMagnitude \sa setInitialPositionLLA \sa initialPositionLLA
*/
bool XsDevice::setGravityMagnitude(double mag)
{
	(void)mag;
	return false;
}

/*! \brief Reinitialize the XsDevice
	\details This function will read all configuration details freshly from the device and will
	reinitialize all filters. Especially when you have made changes to the device configuration outside
	XDA or through sendCustomMessage() it is advisable to call this function so XDA will show the
	correct state of the device.
	\returns true if the device was successfully reinitialized
	\note The device itself is not reset, but will be put in config mode while the settings are being
	updated.
*/
bool XsDevice::reinitialize()
{
	return false;
}

/*! \brief Gets the filter profile in use for computing orientations on the host PC
	\returns The filter profile in use when computing orientations is done on the PC
	\sa setXdaFilterProfile \sa onboardFilterProfile
*/
XsFilterProfile XsDevice::xdaFilterProfile() const
{
	return XsFilterProfile();
}

/*! \brief Sets the filter profile to use for computing orientations on the host PC
	\details When computing orientation data, there is a choice of filter profiles. This function can
	be used	to select the appropriate one. By default XDA will attempt to match the software filter profile
	to the configured hardware filter profile when detecting a new device.
	\param profileType The filter profile type to use. This can be chosen from the list returned by
		availableXdaFilterProfiles()
	\returns true if the filter profile was successfully changed
	\note When reading from a file, make sure to call resetLogFileReadPosition() and possibly
	loadLogFile() after changing the filter profile to make sure all cached data is recomputed.
	\sa availableXdaFilterProfiles \sa xdaFilterProfile \sa setOnboardFilterProfile
*/
bool XsDevice::setXdaFilterProfile(int profileType)
{
	(void)profileType;
	return false;
}

/*! \brief Sets the filter profile to use for computing orientations on the host PC
	\details When computing orientation data, there is a choice of filter profiles. This function can
	be used	to select the appropriate one. By default XDA will attempt to match the software filter profile
	to the configured hardware filter profile when detecting a new device.
	\param profileType The filter profile type to use. This can be chosen from the list returned by
		availableXdaFilterProfiles()
	\returns true if the filter profile was successfully changed
	\note When reading from a file, make sure to call resetLogFileReadPosition() and possibly
	loadLogFile() after changing the filter profile to make sure all cached data is recomputed.
	\sa availableXdaFilterProfiles \sa xdaFilterProfile \sa setOnboardFilterProfile
*/
bool XsDevice::setXdaFilterProfile(XsString const& profileType)
{
	(void)profileType;
	return false;
}

/*! \brief Gets the filter profile in use by the device for computing orientations
	\returns The filter profile in use when computing orientations is done on the device
	\sa setOnboardFilterProfile \sa xdaFilterProfile
*/
XsFilterProfile XsDevice::onboardFilterProfile() const
{
	return XsFilterProfile();
}

/*! \brief Sets the filter profile to use for computing orientations on the device
	\details When computing orientation data, there is a choice of filter profiles. This function can
	be used to select the appropriate one.
	\param profileType The filter profile type to use. This can be chosen from the list returned by
		availableOnboardFilterProfiles()
	\returns true if the filter profile was successfully changed
	\sa availableOnboardFilterProfiles \sa onboardFilterProfile \sa setXdaFilterProfile
*/
bool XsDevice::setOnboardFilterProfile(int profileType)
{
	(void)profileType;
	return false;
}

/*! \brief Sets the filter profile to use for computing orientations on the device
	\details When computing orientation data, there is a choice of filter profiles. This function can
	be used to select the appropriate one.
	\param profileType The filter profile type to use. This can be chosen from the list returned by
		availableOnboardFilterProfiles()
	\returns true if the filter profile was successfully changed
	\sa availableOnboardFilterProfiles \sa onboardFilterProfile \sa setXdaFilterProfile
*/
bool XsDevice::setOnboardFilterProfile(XsString const& profileType)
{
	(void)profileType;
	return false;
}

/*!	\brief Replaces profileCurrent by profileNew in the device
	\param profileCurrent The profile that should be replaced
	\param profileNew The new profile
	\return true if successful, false otherwise
	\note The default implementation does nothing as this feature requires the full XDA
*/
bool XsDevice::replaceFilterProfile(XsFilterProfile const& profileCurrent, XsFilterProfile const& profileNew)
{
	(void)profileCurrent;
	(void)profileNew;
	return false;
}

/*! \brief Return the list of filter profiles available on the device
	\returns The list of filter profiles available for computing orientations on the device
	\sa availableXdaFilterProfiles \sa onboardFilterProfile \sa setOnboardFilterProfile
*/
XsFilterProfileArray XsDevice::availableOnboardFilterProfiles() const
{
	return XsFilterProfileArray();
}

/*! \brief Return the list of filter profiles available on the host PC
	\returns The list of filter profiles available for computing orientations on the host PC
	\sa availableOnboardFilterProfiles \sa xdaFilterProfile \sa setXdaFilterProfile
*/
XsFilterProfileArray XsDevice::availableXdaFilterProfiles() const
{
	return XsFilterProfileArray();
}

/*! \brief Returns the maximum official value of the accelerometers in the device
	\details The actual official range is -accelerometerRange() .. accelerometerRange(). The device may
	send out higher values than this for extreme movements, but then the data quality can not be
	guaranteed.
	\returns The maximum value of the accelerometers in m/s^2
*/
double XsDevice::accelerometerRange() const
{
	return 0;
}

/*! \brief Returns the maximum official value of the gyroscopes in the device
	\details The actual official range is -gyroscopeRange() .. gyroscopeRange(). The device may
	send out higher values than this for extreme movements, but then the data quality can not be
	guaranteed.
	\returns The maximum value of the gyroscopes in degrees/s
*/
double XsDevice::gyroscopeRange() const
{
	return 0;
}

/*! \brief Set the no rotation period to \a duration
	\details This function can be called in both config and measurement modes. In config mode it
	specifies the duration that the device is considered to be stationary as soon as it enters
	measurement mode. In measurement mode, it specifies the duration that the device is considered to
	be stationary, starting immediately.

	During the stationary period, the gyroscope biases are measured, giving better performance.
	\param duration The desired stationary duration in seconds
	\returns true if the no rotation command was accepted by the device
	\sa isInitialBiasUpdateEnabled \sa setInitialBiasUpdateEnabled
*/
bool XsDevice::setNoRotation(uint16_t duration)
{
	(void)duration;
	return false;
}

/*! \brief Let the user indicate that he is starting the representative motion
	for the In-Run Compass Calibration
	\returns true if the start indication was successfully sent to the device
*/
bool XsDevice::startRepresentativeMotion()
{
	return false;
}

/*! \brief Retrieves the active representative motion state
	for the In-Run Compass Calibration
	\returns true if the reprensentation motion state is active, false otherwise
*/
bool XsDevice::representativeMotionState()
{
	return false;
}

/*! \brief Let the user indicate that he stopped the representative motion

	for the In-Run Compass Calibration
	\returns A struct containing the results of the last In-Run Compass Calibration
*/
XsIccRepMotionResult XsDevice::stopRepresentativeMotion()
{
	return XsIccRepMotionResult();
}

/*! \brief Store the onboard ICC results for use by the device
   \returns true if the store was successful
*/
bool XsDevice::storeIccResults()
{
	return false;
}

/*!	\brief Gets the 'Latitude Longitude Altitude' setting of the device
	\details The Latitude Longitude Altitude contains the location on earth where the measurements are
	done. Setting this value allows for more accurate measurements.
	Note: this XDA data type is the setting initialPositionLLA, which is set by setInitialPositionLLA.
	It's value is therefore static. Use LatitudeLongitude to retrieve the live position data from the MTi.
	\returns lla The desired 'Latitude Longitude Altitude' setting for the device.
	\sa setInitialPositionLLA \sa gravityMagnitude
*/
XsVector XsDevice::initialPositionLLA() const
{
	return XsVector();
}

/*!	\brief Sets the 'Latitude Longitude Altitude' setting of the device to the given \a vector.
	\details The Latitude Longitude Altitude contains the location on earth where the measurements are
	done. Setting this value allows for more accurate measurements. The default gravity magnitude and
	earth magnetic field are computed form this value.
	\param lla The desired 'Latitude Longitude Altitude' setting for the device. This should be a
	3-element vector.
	\returns true if the Latitude Longitude Altitude was successfully written
	\note When GNSS is available, this value is automatically updated with the last known position when
	the device is put in config mode after measurement.
	\sa initialPositionLLA \sa labMagneticField \sa gravityMagnitude
*/
bool XsDevice::setInitialPositionLLA(const XsVector& lla)
{
	(void)lla;
	return false;
}

/*!	\brief Gets the 'UTC Time' setting of the device.
	\details Gets the UTC time in the device.
	\returns The current UTC time of the sensor
*/
XsTimeInfo XsDevice::utcTime() const
{
	return XsTimeInfo();
}

/*!	\brief Sets the 'UTC Time' setting of the device to the given \a time.
	\details Sets the UTC time in the device. Setting this value allows for more accurate measurements.
	\param time The current time in UTC format
	\returns true if the UTC Time was successfully written
	\note When GNSS is available, this value is automatically updated with the last known UTC time when
	the device is put in config mode after measurement.
*/
bool XsDevice::setUtcTime(const XsTimeInfo& time)
{
	(void)time;
	return false;
}

/*! \brief Returns the transmission delay used for RS485 transmissions
	\details See the low level documentation for more information on this function.
	\returns The currently configured RS485 transmission delay
*/
uint16_t XsDevice::rs485TransmissionDelay() const
{
	return 0;
}

/*! \brief Set the transmission delay used for RS485 transmissions
	\details See the low level documentation for more information on this function.
	\param delay The desired delay
	\returns true if the device was successfully updated
*/
bool XsDevice::setRs485TransmissionDelay(uint16_t delay)
{
	(void)delay;
	return false;
}

/*! \brief Run the self test for the device
	\details All Xsens devices have limited self-diagnostic functionality, which can be triggered
	by calling this function. The device automatically does some self tests during startup, but
	this function returns more information.
	\returns Results of the test
	\note This function is blocking and can take a few 100 ms
*/
XsSelfTestResult XsDevice::runSelfTest()
{
	return XsSelfTestResult();
}

/*! \brief Request data when configured in legacy mode with infinite skip factor
	\details When configured in legacy mode and an output skip factor of 0xFFFF, the device will not
	send data by itself, but will instead send the latest data after receiving an explicit request.
	This function is that request. After the request, the normal callback mechanism will take over.
	\returns true if the message was successfully sent.
*/
bool XsDevice::requestData()
{
	return false;
}

/*!	\brief Store orientation filter state in the device
	\details Use this function when the filters for the device have stabilized to store the
	current biases in the device. The benefit is that on the next startup the filter
	will stabilize quicker. However, the stored biases depend on temperature and
	other external parameters, so the stored values will remain correct for only a
	short time.
	\returns true if the filter state was saved, false otherwise
*/
bool XsDevice::storeFilterState()
{
	return false;
}

// MTix device
/*! \brief Returns if the device does gyroscope bias estimation when switching to measurement mode
	\details When this option is enabled, the device will automatically run the 'no rotation'
	algorithm every time it switches to measurement mode.
	\returns true if the option is enabled
	\sa setNoRotation \sa setInitialBiasUpdateEnabled
*/
bool XsDevice::isInitialBiasUpdateEnabled() const
{
	return false;
}

/*! \brief Set if the device does gyroscope bias estimation when switching to measurement mode
	\details When this option is enabled, the device will automatically run the 'no rotation'
	algorithm every time it switches to measurement mode.
	\param enable true to enable the option, false to disable it
	\returns true if the device was successfully update
	\sa setNoRotation \sa isInitialBiasUpdateEnabled
*/
bool XsDevice::setInitialBiasUpdateEnabled(bool enable)
{
	(void)enable;
	return false;
}

/*! \brief Returns if the fixed gravity value should be used or if it should be computed from the
	initialPositionLLA value.
	\returns true if the option is enabled
	\sa setFixedGravityEnabled \sa gravityMagnitude \sa initialPositionLLA
*/
bool XsDevice::isFixedGravityEnabled() const
{
	return false;
}

/*! \brief Sets whether the fixed gravity value should be used or if it should be computed from the
	initialPositionLLA value.
	\param enable true to use fixed gravity, false to compute from initialPositionLLA
	\returns true if the device was successfully update
	\sa isFixedGravityEnabled \sa setGravityMagnitude \sa setInitialPositionLLA
*/
bool XsDevice::setFixedGravityEnabled(bool enable)
{
	(void)enable;
	return false;
}

/*! \cond XS_INTERNAL */
/*! \brief Sets a lever arm vector
	\param arm The lever arm vector
	\returns True if successful
*/
bool XsDevice::setLeverArm(const XsVector &arm)
{
	(void)arm;
	return false;
}

/*! \returns The lever arm vector. */
XsVector XsDevice::leverArm() const
{
	return XsVector();
}

/*! \brief Requests UTC time. */
bool XsDevice::requestUtcTime()
{
	return false;
}
/*! \endcond */

// Mtw device
/*! \brief Request the battery level from the device
	\details This is an asynchronous operation. The Awinda station or MTw sends the battery level when
	possible. For devices in wired mode the \sa batteryLevel() function can be called without calling this
	function first.
	\returns true If the battery level request was successfully sent
*/
bool XsDevice::requestBatteryLevel()
{
	return false;
}

/*! \brief Requests the time the battery level was last updated
	\returns the XsTimeStamp the battery level was last set
*/
XsTimeStamp XsDevice::batteryLevelTime()
{
	return 0;
}

/*! \brief Enable or disable the transport mode for the device
	\details The MTw has a "wake up by motion" feature that requires some power and can cause
	unnecessary wakeups when transporting the device. This function can be used to put the device in
	"transport mode", which effectively disables the motion wake up feature until the device is plugged
	into something or the transport mode is explicitly disabled by this function again.
	\param transportModeEnabled true to enable transport mode (which disables the motion wakeup)
	\returns true if the device was successfully put in transport mode (or taken out of it)
	\note MTw only
*/
bool XsDevice::setTransportMode(bool transportModeEnabled)
{
	(void)transportModeEnabled;
	return false;
}

/*! \brief Returns the current state of the transport mode feature
	\returns true if tranport mode is currently enabled
	\sa setTransportMode
*/
bool XsDevice::transportMode()
{
	return false;
}

/*! \brief Returns if the device is outputting data in string mode
	\details In string mode only NMEA packets are transmitted at the legacy update rate
	\returns true if the device is configured for string mode output.
*/
bool XsDevice::isInStringOutputMode() const
{
	return stringOutputType() != 0;
}

/*! \brief Returns whether the device uses legacy device mode
	\returns True if the legacy period, outputmode, outputsettings or string reports are used
*/
bool XsDevice::usesLegacyDeviceMode() const
{
	return isInStringOutputMode();
}

/*! \cond XS_INTERNAL */
/*! \brief Override the log interface
	\note XsDevice takes control of the supplied pointer
*/
void XsDevice::useLogInterface(DataLogger* logger)
{
	m_logFileInterface = logger;
}
/*! \endcond */

/*! \brief Load a complete logfile
	\details Load the opened log file completely. This function loads all data from
	the open logfile in a separate thread, generating onProgressUpdated callbacks.
	This function will return true if the reading was scheduled.
	\returns true if the threaded loading was successfully started
	\sa onProgressUpdated
*/
bool XsDevice::loadLogFile()
{
	Communicator* comm = communicator();
	if (!comm)
		return false;

	if (!comm->isReadingFromFile())
		return false;

	comm->loadLogFile(this);
	return true;
}

/*! \brief Aborts loading a logfile
	\returns true if loading is aborted successfully
	\note if no file was currently loading returns false
*/
bool XsDevice::abortLoadLogFile()
{
	Communicator* comm = communicator();
	if (comm && comm->isReadingFromFile())
	{
		comm->abortLoadLogFile();
		comm->waitForLastTaskCompletion();
		return true;
	}
	return false;
}

/*!	\brief Get the name of the log file the device is reading from.
	\details Returns an empty string when not in file mode.
	\returns The name of the logfile
*/
XsString XsDevice::logFileName() const
{
	if (!isReadingFromFile())
	{
		if (logFileInterface())
		{
			MtbDataLogger const* mtb = dynamic_cast<MtbDataLogger const*>(logFileInterface());
			if (mtb != nullptr)
				return mtb->filename();
		}
		return XsString();
	}

	Communicator *object = communicator();
	if (!object)
		return XsString();
	return object->logFileName();
}

/*!	\brief Get the maximum update rate for the device
	\returns The maximum update rate of the device
	\sa supportedUpdateRates
*/
int XsDevice::maximumUpdateRate() const
{
	// Get the supported update rates
	std::vector<int> frameRates = supportedUpdateRates();

	// Determine the maximum value
	std::vector<int>::iterator p = std::max_element(frameRates.begin(), frameRates.end());
	if (p != frameRates.end())
		return *p;
	else
		return 0;
}

/*!	\brief Get the number of packets currently waiting in the slow data cache for the device based
	\returns The highest received packet ID minus the last reported packet id in the slow data callback
*/
int XsDevice::recordingQueueLength() const
{
	LockGuarded lockG(&m_deviceMutex);

	if (m_dataCache.empty())
		return 0;

	return (int) (m_dataCache.rbegin()->first - latestBufferedPacketId());
}

/*!	\brief Get the number of items currently in the slow data cache for the device
	\returns The actual number of items in the cache, which may contain huge gaps in packet ids
*/
int XsDevice::cacheSize() const
{
	LockGuarded lockG(&m_deviceMutex);
	return (int) m_dataCache.size();
}


/*!	\brief Get all the current synchronization settings of the device
	\details This function is a generic way of requesting the synchonization options of a device,
	since not all devices support the same synchronization functionality.
	\returns The list of synchronization settings configured for the device
	\sa setSyncSettings
*/
XsSyncSettingArray XsDevice::syncSettings() const
{
	return XsSyncSettingArray();
}

/*! \brief Set the synchronization settings of the device
	\details This function can be used to set all the synchronization options of the device at once.
	It is translated into device-specific commands by XDA, since not all devices support the same
	synchronization functionality.
	\param settingList The list of synchronization settings to set. An empty list will clear all
	synchronization settings.
	\returns true if the device was successfully updated
	\sa syncSettings
*/
bool XsDevice::setSyncSettings(const XsSyncSettingArray& settingList)
{
	(void)settingList;
	return false;
}

/*!	\brief Get all supported synchronization settings available on the device.
	\details This function provides a list of the available synchronization settings of the device,
	since not all devices support the same synchronization functionality.
	Every XsSyncSetting element in the list defines one function and line setting, with supported parameters.
	If the same function support multiple settings (i.e. multiple lines),
	then the list will contains multiple items with the same function name, but with different line settings.
	For easier use, same functions must be listed next to eachother, so each function settings
	in the list will be grouped.
	Properties, others then m_function and m_line are set to 0 if not supported or 1 if supported by the device.
	\returns The list of synchronization settings supported by the device. Each settings grouped by functions.
*/
XsSyncSettingArray XsDevice::supportedSyncSettings() const
{
	return Synchronization::supportedSyncSettings(m_deviceId);
}

/*! \brief Perform an orientation reset on the device using the given \a resetMethod
	\details This function schedules an orientation reset command to be applied in the first available
	orientation filter update.
	\param resetmethod The requested orientation reset method.
	\returns true if the orientation reset was successfully scheduled
	\note XRM_StoreAlignmentMatrix can only be used in config mode, the others only in measurement mode
*/
bool XsDevice::resetOrientation(XsResetMethod resetmethod)
{
	return scheduleOrientationReset(resetmethod);
}

/*!	\brief Set the read position of the open log file to the start of the file.
	\details If software filtering is enabled, the appropriate filters will be restarted
	as if the file was just opened.
	\returns true if the read position was successfully reset to the start of the file
	\note This is a low-level file operation.
*/
bool XsDevice::resetLogFileReadPosition()
{
	JLDEBUGG("");

	Communicator* comm = communicator();
	if (!comm || !comm->isReadingFromFile())
		return false;

	comm->resetLogFileReadPosition();

	clearDataCache();
	resetPacketStamping();
	return true;
}

/*!	\brief Get the size of the log file the device is reading from
	\details If the function encounters an error the function returns 0.
	\returns The size of the log file or 0
	\note This is a low-level file operation.
	\sa logFileReadPosition
*/
XsFilePos XsDevice::logFileSize() const
{
	JLDEBUGG("");

	Communicator* comm = communicator();
	if (!comm || !comm->isReadingFromFile())
		return 0;

	return comm->logFileSize();
}

/*!	\brief Get the current read position of the open log file
	\details If the function encounters an error the function returns -1.
	\returns The current read position (in bytes) from the start of the file or -1
	\note This is a low-level file operation.
	\sa logFileSize
*/
XsFilePos XsDevice::logFileReadPosition() const
{
	JLDEBUGG("");

	Communicator* comm = communicator();
	if (!comm || !comm->isReadingFromFile())
		return -1;

	return comm->logFileReadPosition();
}

/*!	\brief Updates the cached device information for all devices connected to this port
	\details This function can only be called in config mode. XDA caches all device information to
	prevent unnecessary communication with the device. When some configuration has changed without
	XDA knowing about it (through sendCustomMessage() for example), it may be necessary to tell XDA
	to refresh its cached information by calling this function.
	\returns true if the cached information was updated successfully
*/
bool XsDevice::updateCachedDeviceInformation()
{
	JLDEBUGG("");

	if (isMeasuring())
		return false;

	return initialize();
}
///@} end Sensor Data

/*! \cond XS_INTERNAL */
/*!	\brief Initializes the device using the supplied filter profiles
	\returns True if successful
*/
bool XsDevice::initialize()
{
	m_isInitialized = true;
	return true;
}
/*! \endcond */

/*! \brief Enable an additional communication protocol when reading messages
	\param protocol The type of protocol-support to add.
	\returns true if the addition was successful
	\note This is a per port or per file setting
*/
bool XsDevice::enableProtocol(XsProtocolType protocol)
{
	Communicator* comm = communicator();
	if (!comm)
		return false;

	switch (protocol)
	{
	case XPT_Xbus:
		comm->addProtocolHandler(new ProtocolHandler());
		return true;
	case XPT_Nmea:
		comm->addProtocolHandler(new nmea::ProtocolHandler());
		return true;
	default:
		return false;	// unknown type
	}
}

/*! \brief Disable a communication protocol previously added by XsDevice::enableProtocol
	\param protocol The type of protocol-support to remove.
	\returns true if the removal was successful
	\note This is a per port or per file setting
*/
bool XsDevice::disableProtocol(XsProtocolType protocol)
{
	Communicator* comm = communicator();
	if (!comm)
		return false;

	switch (protocol)
	{
	case XPT_Xbus:
		comm->removeProtocolHandler(XPT_Xbus);
		return true;
	case XPT_Nmea:
		comm->removeProtocolHandler(XPT_Nmea);
		return true;
	default:
		return false;	// unknown type
	}
}

/*! \returns true when a protocol with type \a type has been added
	\param[in] protocol The protocol type to check
*/
bool XsDevice::isProtocolEnabled(XsProtocolType protocol) const
{
	Communicator* comm = communicator();
	if (!comm)
		return false;
	return comm->hasProtocol(protocol);
}

/*! \brief Returns the sample period for string output
	\returns The 'sample period' setting of the device for string output
*/
uint16_t XsDevice::stringSamplePeriod() const
{
	return 0xFFFF;
}

/*! \brief Returns the skipfactor for string output
	\returns The 'output skip factor' setting of the device for string output
*/
uint16_t XsDevice::stringSkipFactor() const
{
	return 0xFFFF;
}

/*! \brief Returns the string output type
	\returns The 'string output type' setting of the device
	\sa setStringOutputType
*/
uint16_t XsDevice::stringOutputType() const
{
	return 0x0000;
}

/*!	\brief Returns the operational mode
	\returns The current operational mode of the device
*/
XsOperationalMode XsDevice::operationalMode() const
{
	return XOP_LiveStream;
}

/*!	\brief Set the device in the given operational mode.
	\param mode: Desired operional mode.
	\returns True when successful, false otherwise.
*/
bool XsDevice::setOperationalMode(XsOperationalMode mode)
{
	(void)mode;
	return true;
}

/*! \brief Returns the currently configured output of the device
	\returns The output configuration of the device
*/
XsOutputConfigurationArray XsDevice::outputConfiguration() const
{
	return XsOutputConfigurationArray();
}

/*! \brief Returns the currently configured CAN output of the device
	\returns The can output configuration of the device
*/
XsCanOutputConfigurationArray XsDevice::canOutputConfiguration() const
{
	return XsCanOutputConfigurationArray();
}

/*! \brief Returns the currently configured CAN configuration of the device
   \return The can configuration of the device
*/
uint32_t XsDevice::canConfiguration() const
{
	return 0;
}

/*! \brief Enable and disable processing options
	\details These options are used to specify whether XDA should compute certain kinds of data from
	available other data and what data-retention policy to use. On a system with limited
	resources it may be useful to limit the processing and data retention done by XDA. By default XDA will
	do all processing it can do, but retain as little data as possible.
	In case of conflict, \a enable supersedes \a disable.
	\param enable A logically OR'ed combination of XsOptions to enable
	\param disable A logically OR'ed combination of XsOptions to disable
	\sa areOptionsEnabled
*/
void XsDevice::setOptions(XsOption enable, XsOption disable)
{
	if (!communicator()->allowReprocessing())
	{
		enable = enable & ~(XSO_Calibrate | XSO_Orientation);
		disable = disable | (XSO_Calibrate | XSO_Orientation);
	}

	XsOption upd = XsOption_purify((XsOption) ((m_options & ~disable) | enable));
	static const XsOption mask = XSO_KeepLastLiveData | XSO_RetainLiveData | XSO_RetainBufferedData;

	// this has to be done in this order to prevent timing issues
	bool clr = ((m_options & mask) != (upd & mask));
	m_options = upd;
	if (clr)
		clearExternalPacketCaches();
}

/*! \brief Returns true when all the specified processing options are enabled
	\param options The options to check
	\returns true if the options are enabled
	\sa setOptions
*/
bool XsDevice::areOptionsEnabled(XsOption options) const
{
	return (m_options & options) == options;
}

/*! \brief Return the currently enabled options
	\return The options that are enabled for this device and its child devices (if any)
	\sa setOptions \sa areOptionsEnabled
*/
XsOption XsDevice::getOptions() const
{
	return m_options;
}

/*! \brief Return the product code of the device
	\returns The product code of the device
*/
XsString XsDevice::productCode() const
{
	return XsString();
}

/*!	\brief Return the hardware version of the device
	\returns The hardware version of the device
*/
XsVersion XsDevice::hardwareVersion() const
{
	return XsVersion();
}

/*! \brief Set the radio channel to use for wireless communication
	\details This function can be used to enable or disable the radio of an Awinda Station.
	\param channel A valid channel number in the range [11..25] or -1 to disable the radio
	\returns true if the radio was successfully reconfigured
	\note Awinda Station only
*/
bool XsDevice::enableRadio(int channel)
{
	(void)channel;
	return false;
}

/*! \brief Disables the radio for this station, resetting all children to disconnected state
	\returns true if the radio was successfully disabled
	\note Awinda Station only
*/
bool XsDevice::disableRadio()
{
	return false;
}

/*! \brief Returns if the radio is enabled
	\returns true if the radio is enabled
	\note Awinda Sation only
*/
bool XsDevice::isRadioEnabled() const
{
	return false;
}

/*! \brief Returns the radio channel used for wireless communication
	\returns The radio channel used for wireless communication or -1 if the radio is disabled
	\note Awinda Sation only
*/
int XsDevice::radioChannel() const
{
	return -1;
}

#if 0
/*! \brief Returns a quality indication for the current wireless communication
	\returns A quality indication in the range [0-100] where 100 is the best quality. If the function
	return -1, no radio quality indication could be given.
	\note Awinda Station only
*/
int XsDevice::radioQualityIndication() const
{
	return -1;
}
#endif

/*! \cond XS_INTERNAL */
/*! \brief Handles master indication message.
	\param message The message to handle.
*/
void XsDevice::handleMasterIndication(const XsMessage &message)
{
	onNonDataMessage(this, &message);
}
/*! \endcond */

/*! \brief Abort the wireless flushing operation and finalize the recording
	\returns true if no flushing is in progress when the function exits
	\note Awinda Station only
*/
bool XsDevice::abortFlushing()
{
	return true;
}

/*! \brief Accept connections from the device on the parent/master device
	\details This function can be used to accept connections from a device that has been rejected.
	Call this function from within the onConnectivityChanged callback.
	\returns true if the device will be accepted next time it tries to connect
	\note MTw rejected to Awinda Station only
*/
bool XsDevice::acceptConnection()
{
	return false;
}

/*! \brief Reject connections from the device on the parent/master device
	\details This function can be used to reject connections from a device that has connected.
	This function can be called from within the onConnectivityChanged callback or at other times
	when a device is connected.
	\returns true if the device will be rejected next time it tries to connect
	\note After the function returns, this XsDevice should no longer be used.
	\note MTw connected to Awinda Station only
	\sa rejectReason
*/
bool XsDevice::rejectConnection()
{
	return false;
}

/*! \brief Returns the wireless priority of the device
	\returns The wireless priority of the device or 0 if it has none.
	\note MTw connected to Awinda Station only
*/
int XsDevice::wirelessPriority() const
{
	return 0;
}

/*! \brief Sets the wireless priority of the device
	\param priority The desired wireless priority of the device in the range 0-255.
	\returns true if the wireless priority has been successfully updated
	\note MTw connected to Awinda Station only
*/
bool XsDevice::setWirelessPriority(int priority)
{
	(void)priority;
	return false;
}

/*! \cond XS_INTERNAL */
/*!	\brief Sets the connectivity state to \a newState if different than the old state.
	\param[in] newState the new state
*/
void XsDevice::updateConnectivityState(XsConnectivityState newState)
{
	if (newState != m_connectivity)
	{
		JLDEBUGG(deviceId() << " newState " << newState << " oldState " << m_connectivity);
		m_connectivity = newState;
		onConnectivityChanged(this, newState);
	}
}

/*! \brief The default connectivity state for newly created devices
 */
XsConnectivityState XsDevice::defaultChildConnectivityState() const
{
	return m_connectivity;
}
/*! \endcond */

/*! \brief Returns the reason why a device's connection was rejected.
	\details This function is typically called from within the onConnectivityChanged callback when
	the connectivity has changed to XCS_Rejected.
	\returns The reason why the connection was rejected
*/
XsRejectReason XsDevice::rejectReason() const
{
	return XRR_Unknown;
}

/*! \brief Returns the last known RSSI value of the device.
	\details RSSI values are only relevant for wireless devices. Since the value is measured passively,
	any time an RSSI value is received by XDA, the last known value is updated.
	\returns The last known biased RSSI value or XS_RSSI_UNKNOWN if no RSSI value is available (yet)
*/
int16_t XsDevice::lastKnownRssi() const
{
	return XS_RSSI_UNKNOWN;
}

/*! \cond XS_INTERNAL */
/*! \brief Set the packet error rate for the device.
	\param per The packet error rate of the device expressed as a percentage.
 */
void XsDevice::setPacketErrorRate(int per)
{
	(void)per;
}
/*! \endcond */

/*!
 * \brief Returns the packet error rate for the for the device.
 *
 * \details The packet error rate indicates the proportion of data packets from
 * the device that are lost or corrupted in some manner over some time window.
 * Depending on the device the packet error rate may be updated actively or
 * passively, and the time window may vary, so packet error rates cannot be
 * compared directly between different types of device.
 *
 * \note Not all devices support packet error rate estimation. Those that don't
 * will always report a 0% packet error rate.
 *
 * \returns The packet error rate as a percentage.
 */
int XsDevice::packetErrorRate() const
{
	return 0;
}

/*! \brief Returns the connectivity state of the device
	\details The connectivity describes how and if the device is connected to XDA.
	\returns The current connectivity of the device
	\sa XsConnectivityState
*/
XsConnectivityState XsDevice::connectivityState() const
{
	return m_connectivity;
}

/*! \brief Check if the device is docked
	\details Checks if device \a dev is docked in this device
	\param dev The device to check
	\return true if the device is docked in this device
*/
bool XsDevice::deviceIsDocked(XsDevice *dev) const
{
	(void)dev;
	return false;
}

/*! \brief The port name of the connection
	\returns The name of the communication port or an empty string if not connected to a communication
	port
*/
XsString XsDevice::portName() const
{
	Communicator* comm = communicator();
	if (!comm)
		return XsString();

	return comm->portInfo().portName();
}

/*! \brief The port information of the connection
	\returns The port information object containing the information of the communication port or an empty structure if not connected to a communication
	port
*/
XsPortInfo XsDevice::portInfo() const
{
	Communicator* comm = communicator();
	if (!comm)
		return XsPortInfo();

	return comm->portInfo();
}

#ifndef XSENS_NO_PORT_NUMBERS
/*! \brief Return the port number of the connection
	\returns The port number or 0 if no port is open that can be identified by a single number
*/
int XsDevice::portNumber() const
{
	Communicator* comm = communicator();
	if (!comm)
		return 0;
	return comm->portInfo().portNumber();
}
#endif

/*! \brief Returns true when the device is initialized
	\returns true when the device has been initialized
*/
bool XsDevice::isInitialized() const
{
	return m_isInitialized;
}

/*! \brief Accepts a device
	\param[in] deviceId The device to accept
	\returns true when the device has been successfully accepted
*/
bool XsDevice::setDeviceAccepted(const XsDeviceId&)
{
	return false;
}

/*! \brief Rejects a device
	\param[in] deviceId The device to reject
	\returns true when the device has been successfully rejected
*/
bool XsDevice::setDeviceRejected(const XsDeviceId&)
{
	return false;
}

/*! \brief Set the access control mode of the master device
	\details The access control mode determines which connections are allowed.
	\param mode The access control mode to use, the choice is between blacklist or whitelist
	\param initialList The initial list to use for the selected access control mode
	\returns true if the access control mode was successfully changed
	\sa accessControlMode() \sa currentAccessControlList
*/
bool XsDevice::setAccessControlMode(XsAccessControlMode, const XsDeviceIdArray&)
{
	return false;
}

/*! \brief Request the access control mode of the master device
	\returns The currently configured access control mode
	\sa setAccessControlMode \sa currentAccessControlList
*/
XsAccessControlMode XsDevice::accessControlMode() const
{
	return XACM_None;
}

/*! \brief Request the access control list of the master device
	\returns The currently configured access control list. This can be either a blacklist or a whitelist.
	\sa setAccessControlMode \sa accessControlMode
*/
XsDeviceIdArray XsDevice::currentAccessControlList() const
{
	return XsDeviceIdArray();
}

/*! \brief Sets the Awinda station to operational state
	\note this is considered an extension to the config state, not a new state.
	\returns true when the awindastation is put in operational mode.
*/
bool XsDevice::makeOperational()
{
	return false;
}

/*! \returns true when the device is operational
	\sa AwindaStationDevice::makeOperational()
*/
bool XsDevice::isOperational() const
{
	return false;
}

/*! \returns true when the device is in Sync Station mode (Awinda Station and Sync Station only) */
bool XsDevice::isInSyncStationMode()
{
	return false;
}

/*! \brief Set the Sync Station mode of the Awinda Station device
	\param enabled true to enable Sync Station mode, false to disable it
	\returns true if successful
*/
bool XsDevice::setSyncStationMode(bool)
{
	return false;
}

/*! \brief Increase reference count of XsDevice pointer
	XsDevice pointers stay alive while reference counter is not zero
	Also increases the reference count of each child device with 1
*/
void XsDevice::addRef()
{
	JLTRACEG(this << " pre: " << m_refCounter.load());
	++m_refCounter;
}

/*! \brief The current reference counter
	\returns The current reference count
*/
XsSize XsDevice::refCounter() const
{
	return m_refCounter.load();
}

/*! \brief Decrease this XsDevices reference counter with 1
	Also decreases the reference count of each child with 1
	- If it is a child device, it will delete itself when the reference count reaches zero.
	  It will also remove itself from its master's child list and ask the master if it can be deleted
	- If it is a master device, it will delete itself when the reference count reaches zero and the reference count
	  of all children is zero.
*/
void XsDevice::removeRef()
{
	JLTRACEG(this << " pre: " << m_refCounter.load());

	// Do not decrease when already at 0 ref count
	// In this case, the device is just waiting for all children to be dereferenced
	if (m_refCounter.load() != 0)
	{
		--m_refCounter;
		assert(m_refCounter.load() >= 0);
	}

	// Always check if this device can be removed
	removeIfNoRefs();
}

/*! \cond XS_INTERNAL */
/*!	\brief Writes an EMTS page to the device
	\param data The data buffer
	\param pageNr The page number
	\param bankNr The bank number
	\returns True if succesful
*/
bool XsDevice::writeEmtsPage(uint8_t const* data, int pageNr, int bankNr)
{
	(void)data;
	(void)pageNr;
	(void)bankNr;
	return false;
}

/*! \brief Delete itself when all references and child references are zero
*/
void XsDevice::removeIfNoRefs()
{
	// If any child has refs, no need to delete this device
	LockSuspendable lock(&m_deviceMutex, LS_Write);
	if (childCount())
		return;
	//for (XsDevice* dev:m_children)
	//	if (dev->refCounter() > 0)
	//		return;

	if (m_refCounter.load() == 0)
	{
		lock.unlock();
		prepareForTermination();
		delete this;
	}
}

/*! \brief Return true if the message looks sane */
bool XsDevice::messageLooksSane(const XsMessage &msg) const
{
	return msg.getBusId() == XS_BID_MASTER;
}

void XsDevice::onMessageSent(const XsMessage &msg)
{
	onMessageSentToDevice(this, &msg);
}

void XsDevice::onMessageReceived(const XsMessage &msg)
{
	onMessageReceivedFromDevice(this, &msg);
}

void XsDevice::onMessageDetected(XsProtocolType type, const XsByteArray &rawMessage)
{
	CallbackManagerXda::onMessageDetected(this, type, &rawMessage);
}

void XsDevice::onSessionRestarted()
{
}

void XsDevice::onConnectionLost()
{
	updateConnectivityState(XCS_Disconnected);
}

void XsDevice::onEofReached()
{
	LockGuarded locky(&m_deviceMutex);
	setStopRecordingPacketId(latestLivePacketId());
	endRecordingStream();
}

void XsDevice::onWirelessConnectionLost()
{
	updateConnectivityState(XCS_Disconnected);
}

int64_t XsDevice::latestLivePacketId() const
{
	LockGuarded locky(&m_deviceMutex);
	return latestLivePacketConst().packetId();
}

int64_t XsDevice::latestBufferedPacketId() const
{
	LockGuarded locky(&m_deviceMutex);
	return latestBufferedPacketConst().packetId();
}

/*! \return true if the message should be written to log file
	\param msg The message to write
	\details We have some cases where we don't want every child device to write its messages to log file (XM/AbmClockMaster)
	\returns True if successful
*/
bool XsDevice::shouldWriteMessageToLogFile(const XsMessage &msg) const
{
	return shouldWriteMessageToLogFile(this, msg);
}

/*! \return true if \a child should be writing to log file
	\param dev The child device
	\param message The message to write
	\details We have some cases where we don't want every child device to write its messages to log file (XM/AbmClockMaster)
	\returns True if successful
*/
bool XsDevice::shouldWriteMessageToLogFile(const XsDevice *dev, const XsMessage &message) const
{
	(void)dev;
	(void)message;
	return true;
}

/*! \brief Send a message and wait for its reply
	\details The expected reply is always the Ack to \a snd's message
	\param snd the message to send
	\returns true if the message was sent and acknowledged, false otherwise
*/
bool XsDevice::doTransaction(const XsMessage &snd) const
{
	return communicator() && communicator()->doTransaction(snd);
}

/*! \brief Send a message and wait for its reply
	\details The expected reply is always the Ack to \a snd's message
	\param snd the message to send
	\param timeout the timeout to use
	\returns true if the message was sent and acknowledged, false otherwise
*/
bool XsDevice::doTransaction(const XsMessage &snd, uint32_t timeout) const
{
	return communicator() && communicator()->doTransaction(snd, timeout);
}

/*! \brief Send a message and wait for its reply
	\details The expected reply is always the Ack to \a snd's message
	\param snd the message to send
	\param rcv a pointer to a receive message. The contents are only valid if true was returned.
	\returns true if the message was sent and acknowledged, false otherwise
*/
bool XsDevice::doTransaction(const XsMessage &snd, XsMessage &rcv) const
{
	return communicator() && communicator()->doTransaction(snd, rcv);
}

/*! \brief Send a message and wait for its reply
	\details The expected reply is always the Ack to \a snd's message
	\param snd the message to send
	\param rcv a pointer to a receive message
	\param timeout the timeout to use
	\returns true if the message was sent and acknowledged, false otherwise
*/
bool XsDevice::doTransaction(const XsMessage &snd, XsMessage &rcv, uint32_t timeout) const
{
	return communicator() && communicator()->doTransaction(snd, rcv, timeout);
}
/*! \endcond */

/*! \brief Enable or disable stealth mode
	\details In stealth mode, the MVN hardware will be silent and all LEDs will be dimmed or disabled.
	Some minimal user feedback will remain enabled. The change will be applied immediately to all detected
	systems, resetting the mocap data stream.
	\param enabled Set to true if you wish to enable stealth mode, false to disable it.
	\return true if the setting was successfully updated
*/
bool XsDevice::setStealthMode(bool)
{
	return false;
}

/*! \brief Return the state of the stealth mode setting.
	\return the state of the stealth mode setting.
	\sa setStealthMode
*/
bool XsDevice::stealthMode() const
{
	return false;
}

/*! \brief Return the supported synchronization settings for a specified \a deviceId or deviceId mask
	\param[in] deviceId The device id to request the supported synchronization settings for
	\returns the supported synchronization settings for the specified deviceId
*/
XsSyncSettingArray XsDevice::supportedSyncSettings(XsDeviceId deviceId)
{
	XsSyncSettingArray settings = Synchronization::supportedSyncSettings(deviceId);
	XsSyncSettingArray result;
	for (XsSyncSetting const & setting : settings)
		result.push_back(setting);

	return result;
}

/*! \brief Determines whether the device specified by \a deviceId supports sync settings
	\param[in] deviceId The device id to check
	\returns true when the device specified by \a deviceId supports sync settings
*/
bool XsDevice::supportsSyncSettings(XsDeviceId deviceId)
{
	return Synchronization::supportsSyncSettings(deviceId);
}

/*! \brief Determines whether \a setting1 is compatible with \a setting2 for deviceId \a deviceId
	\param[in] deviceId The device id
	\param[in] setting1 Setting 1
	\param[in] setting2 Setting 2
	\returns true when \a setting1 is compatible with \a setting2 for deviceId \a deviceId
*/
bool XsDevice::isCompatibleSyncSetting(XsDeviceId deviceId, XsSyncSetting const & setting1, XsSyncSetting const & setting2)
{
	return Synchronization::isCompatibleSyncSetting(deviceId, setting1, setting2);
}

/*! \returns the time resolution in microseconds for a device with device id \a deviceId
	For example if the precision is 1 millisecond, 1000 is returned
	\param[in] deviceId The deviceId
*/
unsigned int XsDevice::syncSettingsTimeResolutionInMicroSeconds(XsDeviceId deviceId)
{
	return Synchronization::timeResolutionInMicroseconds(deviceId);
}

/*! \brief Return the number of child-devices this device has. For standalone devices this is always 0
	\returns The number of child devices of the device
*/
int XsDevice::childCount() const
{
	return 0;
}

/*! \brief Return a managed array containing the child-devices this device has. For standalone devices this is always an empty array
	\returns An array of pointers to the child devices of the device
*/
std::vector<XsDevice*> XsDevice::children() const
{
	return std::vector<XsDevice*>();
}

/*! \cond XS_INTERNAL */
/*! \brief Sets a communicator for a device
	\param comm a communicator
*/
void XsDevice::setCommunicator(Communicator* comm)
{
	if (m_communicator)
		m_communicator->destroy();
	m_communicator = comm;
}

/*! \brief Set the frame at which the recording is actually started
*/
void XsDevice::setRecordingStartFrame(uint16_t startFrame)
{
	LockGuarded lock(&m_deviceMutex);
	m_stopRecordingPacketId = -1;
	m_startRecordingPacketId = PacketStamper::calculateLargePacketCounter(startFrame, latestLivePacketId(), PacketStamper::MTSCBOUNDARY);
	JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
}

/*! \brief Set the frame at which the recording is actually stopped
*/
void XsDevice::setRecordingStopFrame(uint16_t stopFrame)
{
	LockGuarded lock(&m_deviceMutex);
	JLDEBUGG(this << " " << deviceId() << " Setting recording stop frame to " << stopFrame << " last known frame is " << latestBufferedPacketId() << " (" << (latestBufferedPacketId() & 0xffff) << ")");
	m_stopRecordingPacketId = PacketStamper::calculateLargePacketCounter(stopFrame, latestBufferedPacketId(), PacketStamper::MTSCBOUNDARY);
	m_stoppedRecordingPacketId = m_stopRecordingPacketId;
	JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
}

/*! \brief Clear the external packet cache
*/
void XsDevice::clearExternalPacketCaches()
{
	LockGuarded lock(&m_deviceMutex);
	for (auto item : m_linearPacketCache)
		delete item;
	m_linearPacketCache.clear();
	m_lastAvailableLiveDataCache->clear();
}

/*! \brief Clear the data cache up till the packet indicated by the start recording frame
*/
void XsDevice::clearCacheToRecordingStart()
{
	LockGuarded lockG(&m_deviceMutex);
	for (auto item : m_dataCache)
	{
		if (item.first < m_startRecordingPacketId)
		{
			delete item.second;
			m_dataCache.erase(item.first);
		}
	}
}

/*! \brief Merge the supplied \a pack into m_lastAvailableLiveDataCache so its data is now available when calling lastAvailableLiveData()
*/
void XsDevice::updateLastAvailableLiveDataCache(XsDataPacket const& pack)
{
	LockGuarded lockG(&m_deviceMutex);
	m_lastAvailableLiveDataCache->merge(pack, true);
}

/*! \brief Add the supplied \a pack to the linear packet cache
	\param pack The packet to retain in the packet cache
 */
void XsDevice::retainPacket(XsDataPacket const& pack)
{
	LockGuarded lockG(&m_deviceMutex);
	if (!m_linearPacketCache.empty() && m_linearPacketCache.back()->packetId() == pack.packetId())
		m_linearPacketCache.back()->merge(pack, true);
	else
	{
		XsDataPacket* tmp = new XsDataPacket(pack);
		m_linearPacketCache.push_back(tmp);
	}
}

/*! \brief Return whether a device reset will remove the COM port connection
	\return True or false
	\note When connected directly to USB, the device cannot reset without losing connection. Then a rescan should be done
*/
bool XsDevice::resetRemovesPort() const
{
	return true;
}
/*! \endcond */

/*! \brief Return the cached data packet with \a index
	\param index The requested index, this does not have to be the same as the packet counter
	\return The requested packet or an empty packet if \a index is out of range
	\note This only works if XSO_RetainLiveData or XSO_RetainBufferedData was set before the data was read
	\sa getDataPacketCount
	\sa setOptions
*/
XsDataPacket XsDevice::getDataPacketByIndex(XsSize index) const
{
	LockGuarded lockG(&m_deviceMutex);
	if (index < m_linearPacketCache.size())
		return *m_linearPacketCache.at(index);
	return XsDataPacket();
}

/*! \brief Return the current size of the retained data packet cache
	\return The current size of the cache
	\sa getDataPacketCount
	\sa setOptions
*/
XsSize XsDevice::getDataPacketCount() const
{
	LockGuarded lockG(&m_deviceMutex);
	return m_linearPacketCache.size();
}

/*! \brief Return the last available live data
	\return A packet containing the latest available live data. This packet will contain the latest data of each
	appropriate type, so it may contain old data mixed with new data if different data comes in at different speeds.
	\note This only works if XSO_KeepLastLiveData is set
	\sa setOptions
*/
XsDataPacket XsDevice::lastAvailableLiveData() const
{
	LockGuarded lockG(&m_deviceMutex);
	return *m_lastAvailableLiveDataCache;
}

/*! \brief Return the first packet in the packet queue or an empty packet if the queue is empty
	\details This function will only return a packet when XSO_RetainLiveData or XSO_RetainBufferedData is specified for the
	device. It will return the first packet in the queue and remove the packet from the queue.
	\return The first packet in the queue or an empty packet if the queue is empty
	\sa setOptions
*/
XsDataPacket XsDevice::takeFirstDataPacketInQueue()
{
	LockGuarded lockG(&m_deviceMutex);
	if (m_linearPacketCache.empty())
		return XsDataPacket();

	std::unique_ptr<XsDataPacket> rv(m_linearPacketCache.at(0));
	m_linearPacketCache.pop_front();
	return *rv;	// copy and delete, hopefully optimizing the copy into a move operation
}

/*! \cond XS_INTERNAL */
/*! \brief Sets start recording packet id
	\param startFrame a start frame value
*/
void XsDevice::setStartRecordingPacketId(int64_t startFrame)
{
	LockGuarded lockG(&m_deviceMutex);
	m_startRecordingPacketId = startFrame;
	ONLYFIRSTMTX2
	JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
}

/*! \brief Sets stop recording packet id
	\param stopFrame a stop frame value
*/
void XsDevice::setStopRecordingPacketId(int64_t stopFrame)
{
	LockGuarded lockG(&m_deviceMutex);
	m_stopRecordingPacketId = stopFrame;
	m_stoppedRecordingPacketId = m_stopRecordingPacketId;
	ONLYFIRSTMTX2
	JLDEBUGG(this << " " << deviceId() << " m_startRecordingPacketId = " << m_startRecordingPacketId << " m_stopRecordingPacketId = " << m_stopRecordingPacketId << " m_stoppedRecordingPacketId = " << m_stoppedRecordingPacketId);
}

/*! \brief Sets skip emts read on initialization
	\param skip Set to true if you want to skip it
*/
void XsDevice::setSkipEmtsReadOnInit(bool skip)
{
	m_skipEmtsReadOnInit = skip;
}

/*! \brief Reads EMTS and device configuration.
	\details EMTS will be stored in \ref m_emtsBlob.
	\note Used in public source to have this data cached for log file header, so MTM can read the file.
	\returns True if successful.
*/
bool XsDevice::readEmtsAndDeviceConfiguration()
{
	if (!gotoConfig())
		return false;

	XsMessage snd(XMID_ReqEmts, 2);
	snd.setBusId(busId());
	snd.setDataByte(0, 0);
	snd.setDataByte(255, 1);

	XsMessage rcv;
	if (!doTransaction(snd, rcv, 10000))
		return false;

	m_emtsBlob = rcv;

	snd.setMessageId(XMID_ReqConfiguration);
	snd.setBusId(busId());

	rcv.clear();
	if (!doTransaction(snd, rcv, 5000))
		return false;

	m_config.readFromMessage(rcv);

	return true;
}

/*! \brief Gets a device recording buffer item count.
	\param lastCompletePacketId an id of last complete packet.
*/
int64_t XsDevice::deviceRecordingBufferItemCount(int64_t& lastCompletePacketId) const
{
	lastCompletePacketId = latestLivePacketId();
	return 0;
}
/*! \endcond */

/*!	\brief Request the size of the interal buffer
	\return Buffer size in number of frames
*/
uint32_t XsDevice::deviceBufferSize()
{
	return 0;
}

/*!	\brief Request the device to set it's internal buffer to the specified size
	\param frames: buffer size in frames
	\return True if the setting was successfully updated
 */
bool XsDevice::setDeviceBufferSize(uint32_t frames)
{
	(void)frames;
	return true;
}

/*! \brief Wait until are known devices are initialized
	\details Container devices such as Awinda Master and Bodypack can have (slightly) delayed initialization
	of child devices after they have been detected. This function can be used to wait for all currently
	detected trackers to have been properly initialized.
*/
void XsDevice::waitForAllDevicesInitialized()
{
	while (!isInitialized());
}

/*! \brief Returns true if the file operation started by loadLogFile is still in progress
	\return true if the file operation started by loadLogFile is still in progress
	\sa loadLogFile \sa waitForLoadLogFileDone
*/
bool XsDevice::isLoadLogFileInProgress() const
{
	Communicator const* comm = communicator();
	if (!comm)
		return false;

	if (!comm->isReadingFromFile())
		return false;

	return comm->isLoadLogFileInProgress();
}

/*! \brief Wait for the file operation started by loadLogFile to complete
	\sa loadLogFile \sa isLoadLogFileInProgress
*/
void XsDevice::waitForLoadLogFileDone() const
{
	while (isLoadLogFileInProgress() && !m_terminationPrepared)
		XsTime::msleep(10);
}

/*! \brief Return the ID of the first packet that should be recorded
	\details This is only valid in Recording or Flushing states
	\return The ID of the first packet that should be recorded
*/
int64_t XsDevice::getStartRecordingPacketId() const
{
	return m_startRecordingPacketId;
}

/*! \brief Return the ID of the last packet that should be recorded
	\details This is only valid in Recording or Flushing states or in Measurement after a recording has finished
	\return The ID of the last packet that should be / was recorded
*/
int64_t XsDevice::getStopRecordingPacketId() const
{
	return m_stopRecordingPacketId == -1 ? m_stoppedRecordingPacketId : m_stopRecordingPacketId;
}

/*!	\brief Sets the given parameter for the device
	\details Settings device parameters is only valid after initialization and before switching the device to be operational.
	\param parameter: a parameter object, corresponding to a row in the table below.
	\return Result value indicating success (XRV_OK) or unsupported with current device or firmware version (XRV_UNSUPPORTED).

	For Awinda stations this function needs to be called before enabling the radio (\a XsDevice::enableRadio).

	<table>
		<tr>
			<th rowspan="2">XsDeviceParameterIdentifier</th>
			<th rowspan="2">Value type/range</th>
			<th rowspan="2">Description</th>
			<th colspan="6">Supported devices</th>
		</tr>
		<tr>
			<th>Awinda 2 (Station, Dongle)</th>
			<th>BodyPack</th>
			<th>MTmk4</th>
			<th>MTw2</th>
			<th>MTx2</th>
			<th>SyncStation</th>
		</tr>
		<tr>
			<td>XDPI_PacketErrorRate</td>
			<td>boolean</td>
			<td>Report packet error rate for each child device.</td>
			<td>Y</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
		</tr>
		<tr>
			<td>XDPI_SyncLossTimeout</td>
			<td>uint16_t</td>
			<td>Network timeout in seconds used by connected child devices.<br/>For Awinda2: this timeout is measured by connected MTw2 devices based on received data from the station.</td>
			<td>Y</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
		</tr>
		<tr>
			<td>XDPI_UplinkTimeout</td>
			<td>uint16_t</td>
			<td>Network timeout in seconds used by the master devices.<br/>For Awinda2: this timeout is measured based on received data from each connected MTw2 individually.</td>
			<td>Y</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
		</tr>
		<tr>
			<td>XDPI_ExtendedBuffer</td>
			<td>bool</td>
			<td>If set, enables the extended buffer on connected MTw2 devices.</td>
			<td>Y</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
			<td>-</td>
		</tr>
	</table>
*/
XsResultValue XsDevice::setDeviceParameter(XsDeviceParameter const& parameter)
{
	(void)parameter;
	return XRV_UNSUPPORTED;
}

/*!	\brief Retrieves the requested parameter's current value
	\details Retrieving device parameters is only valid after initialization
	\param parameter: a parameter object, corresponding to a row in the table listed under \a XsDevice::setParameter
	\return Result value indicating success (XRV_OK) or unsupported with current or current firmware version (XRV_UNSUPPORTED)
	\sa XsDevice::setParameter
*/
XsResultValue XsDevice::deviceParameter(XsDeviceParameter& parameter) const
{
	(void)parameter;
	return XRV_UNSUPPORTED;
}

/*! \brief Returns the device GNSS platform
	\returns The current device GNSS platform
*/
XsGnssPlatform XsDevice::gnssPlatform() const
{
	return XGP_Portable;
}

/*! \brief Set the device GNSS platform
	\param gnssPlatform The GNSS platform that must be set
	\returns true if the device GNSS platform was successfully set
*/
bool XsDevice::setGnssPlatform(XsGnssPlatform gnssPlatform)
{
	XsMessage snd(XMID_SetGnssPlatform, 1);
	snd.setBusId(busId());
	snd.setDataShort((uint16_t)gnssPlatform);

	if (!doTransaction(snd))
		return false;

	return true;
}

/*! \cond XS_INTERNAL */
void XsDevice::reinitializeProcessors()
{
	// intentionally empty, private implementation
}

/*! \brief This function indicates if the device supports retransmissions (at all)
	\details This is required for the handleDataMessage to deal with missed data properly.
	\param packetId The ID of the packet that we want to check
	\return true if a retransmission is expected for the given packet
	\note Devices that return true are expected to handle their retransmissions properly themselves!
*/
bool XsDevice::expectingRetransmissionForPacket(int64_t packetId) const
{
	if (isMasterDevice())
		return false;

	// child devices use their parent's decision
	return master()->expectingRetransmissionForPacket(packetId);
}

bool XsDevice::initializeSoftwareCalibration()
{
	// intentionally empty, private implementation
	return false;
}

void XsDevice::deinitializeSoftwareCalibration()
{
	// intentionally empty, private implementation
}

/*! \brief Returns the first child device (if any) of the master. If there are no children, it will return the master device itself.
*/
XsDevice const* XsDevice::firstChild() const
{
	return this;
}

/*! \brief Tell XDA to interpolate missing items from \a prev to \a pack
	\param pack The latest received packet
	\param prev The previously received packet
	\param packetHandler The function to call with all newly created intermediate and the final packet. packetHandler is expected to take control of its argument.
	\return true if interpolation was successful, false if it was not
	\note The default implementation does nothing
*/
bool XsDevice::interpolateMissingData(XsDataPacket const & pack, XsDataPacket const & prev, std::function<void (XsDataPacket*)> packetHandler)
{
	(void) pack;
	(void) prev;
	(void) packetHandler;
	return false;
}
/*! \endcond */

/*! \brief Tell XDA and the device that any data from before \a firstNewPacketId may be lossy
	\details Tell the device to not request retransmissions of missed data older than the supplied \a firstNewPacketId.
	If \a firstNewPacketId is beyond the end of the recording or beyond the highest received packet ID,
	the lower value is used instead. This means that you can't set this for future packets.
	\param firstNewPacketId The first packet that (if missing) <i>should</i> be retransmitted.
	\note This applies to master devices that support retransmissions only: Awinda and Bodypack.
*/
void XsDevice::discardRetransmissions(int64_t firstNewPacketId)
{
	(void) firstNewPacketId;
}

/*! \brief Returns a bitmask with all the status flags supported by this device
	\details Not all devices support all status flags. When receiving an XsDataPacket with a status in it,
	this can affect how to interpret the flags. Especially with a flag like the self-test it's important
	not to conclude that a device is defective because it is not set when the device doesn't actually support
	this feature.
	\return The bitmask with all the status flags that this device supports
*/
uint32_t XsDevice::supportedStatusFlags() const
{
	// The basic device supports nothing
	return (uint32_t) 0;
}
