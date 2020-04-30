
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

#ifndef XSDEVICE_DEF_H
#define XSDEVICE_DEF_H

#include <xstypes/xssyncrole.h>
#include <xstypes/xsversion.h>
#include "packetstamper.h"
#include "xsdeviceconfiguration.h"
#include "xserrormode.h"
#include <xstypes/xsresetmethod.h>
#include "callbackmanagerxda.h"
#include <xstypes/xsoption.h>
#include "xsrejectreason.h"
#include "communicator.h"
#include "xsalignmentframe.h"
#include "xsaccesscontrolmode.h"
#include "datapacketcache.h"
#include <xstypes/xsdeviceoptionflag.h>
#include "lastresultmanager.h"
#include "xsgnssplatform.h"
#include <functional>
#include "xsoperationalmode.h"

class XSNOEXPORT MtContainer;
class XSNOEXPORT DataLogger;
class XSNOEXPORT PacketProcessor;

//AUTO namespace xstypes {
struct XsString;
struct XsDeviceId;
struct XsPortInfo;
struct XsDataPacket;
struct XsSyncSetting;
struct XsVersion;
struct XsScrData;
struct XsCalibratedData;
struct XsTimeInfo;
struct XsOutputConfigurationArray;
struct XsCanOutputConfigurationArray;
struct XsIntArray;
struct XsMatrix3x3;
struct XsDeviceIdArray;
struct XsDataPacket;
struct XsMessage;
struct XsSyncSettingArray;
struct XsMatrix;
struct XsVector;
struct XsStringArray;
struct XsQuaternion;
struct XsFilterProfileArray;
struct XsOutputConfiguration;
struct XsStringOutputTypeArray;
//AUTO+chdr struct XsException;
//AUTO }

//AUTO namespace xscontroller {
struct XsCallbackPlainC;
struct XsSelfTestResult;
struct XsDeviceParameter;
struct XsIccRepMotionResult;
//AUTO }

#define DebugFileType FILE	// required for autogenerate to correctly parse this

//AUTO namespace xstypes {
struct XsFilterProfile;
//AUTO enum XsBaud;
//AUTO enum XsResultValue;
//AUTO enum XsSyncRole;
//AUTO struct XsIntArray;
//AUTO struct XsOutputConfigurationArray;
//AUTO struct XsCanOutputConfigurationArray;
//AUTO struct XsSyncSettingArray;
//AUTO enum XsXbusMessageId;
//AUTO enum XsFilePos;
//AUTO struct XsFilterProfileArray;
//AUTO enum XsDeviceOptionFlag;
//AUTO enum XsResetMethod;
//AUTO enum XsOption;
//AUTO }

//AUTO namespace xscontroller {
//AUTO struct XsDevicePtrArray;
//AUTO enum XsDeviceState;
//AUTO enum XsErrorMode;
//AUTO struct XsDeviceConfiguration;
//AUTO enum XsProtocolType;
//AUTO enum XsRejectReason;
//AUTO enum XsAlignmentFrame;
//AUTO enum XsOperationalMode;
//AUTO enum XsAccessControlMode;
//AUTO struct XsDeviceParameter;
//AUTO enum XsGnssPlatform;
//AUTO struct XsIccRepMotionResult;
//AUTO }

//AUTO struct XdaConfig;

struct XsDevice : public CallbackManagerXda {
public:
	XSNOEXPORT virtual ~XsDevice();

	virtual void addRef();
	virtual void removeRef();
	XsSize refCounter() const;

	virtual XsDevice *master() const;

	virtual XsDevice *findDevice(XsDeviceId deviceid);
	XsDevice const *findDeviceConst(XsDeviceId deviceid) const;
	virtual int busId() const;
	XsDeviceId deviceId() const;
	virtual XsVersion firmwareVersion() const;
	bool isMasterDevice() const;
	virtual bool isContainerDevice() const;
	bool isInitialized() const;
	bool isStandaloneDevice() const;

	const XsDevice *deviceAtBusIdConst(int busid) const;
	virtual XsDevice *deviceAtBusId(int busid);

	Communicator XSNOEXPORT *communicator() const;
	DataLogger XSNOEXPORT *logFileInterface() const;

	void setGotoConfigOnClose(bool gotoConfigOnClose);

	XsResultValue createLogFile(const XsString &filename);
	virtual bool closeLogFile();

	virtual bool isMeasuring() const;
	virtual bool isRecording() const;
	virtual bool isReadingFromFile() const;

	virtual void XSNOEXPORT checkDataCache();
	virtual void restartFilter();

	XsResultValue lastResult() const;
	XsString lastResultText() const;

	int recordingQueueLength() const;
	int cacheSize() const;

	virtual XsDeviceState deviceState() const;

	static bool supportsSyncSettings(XsDeviceId deviceId);
	static bool isCompatibleSyncSetting(XsDeviceId deviceId, XsSyncSetting const & setting1, XsSyncSetting const & setting2);
	static unsigned int syncSettingsTimeResolutionInMicroSeconds(XsDeviceId deviceId);

#if DOXYGEN
	// Explicit inheritance for generator
	void XSNOCOMEXPORT clearCallbackHandlers(bool chain = true);
	void XSNOCOMEXPORT addCallbackHandler(XsCallbackPlainC* cb, bool chain = true);
	void XSNOCOMEXPORT removeCallbackHandler(XsCallbackPlainC* cb, bool chain = true);
#endif

	//! \brief Compare device ID with that of \a dev \param dev Device to compare against \returns true if \a dev has a higher device ID \sa deviceId()
	bool operator < (const XsDevice& dev) const
		{ return m_deviceId.toInt() < dev.m_deviceId.toInt(); }
	//! \brief Compare device ID with that of \a dev \param dev Device to compare against \returns true if \a dev has the same device ID \sa deviceId()
	bool operator == (const XsDevice& dev) const
		{ return m_deviceId.toInt() == dev.m_deviceId.toInt(); }
	//! \brief Compare device ID with \a devId \param devId DeviceId to compare against \returns true if \a devId is higher than the contained device ID \sa deviceId()
	bool operator < (XsDeviceId devId) const
		{ return m_deviceId.toInt() < devId.toInt(); }
	//! \brief Compare device ID with \a devId \param devId DeviceId to compare against \returns true if \a devId is the same as the contained device ID \sa deviceId()
	bool operator == (XsDeviceId devId) const
		{ return m_deviceId.toInt() == devId.toInt(); }

	XsDeviceConfiguration deviceConfiguration() const;
	XsDeviceConfiguration& XSNOEXPORT deviceConfigurationRef();
	virtual XsDeviceConfiguration const& XSNOEXPORT deviceConfigurationConst() const;

	//! \internal
	XSNOEXPORT template <typename T> T* toType()
		{ return dynamic_cast<T*>(this); }

	virtual bool XSNOEXPORT initialize();
	virtual bool XSNOEXPORT initializeSoftwareCalibration();
	virtual void XSNOEXPORT deinitializeSoftwareCalibration();

	virtual int batteryLevel() const;
	virtual int updateRateForDataIdentifier(XsDataIdentifier dataType) const;
	virtual int updateRateForProcessedDataIdentifier(XsDataIdentifier dataType) const;
	virtual std::vector<int> supportedUpdateRates(XsDataIdentifier dataType = XDI_None) const;
	virtual int maximumUpdateRate() const;
	virtual bool hasDataEnabled(XsDataIdentifier dataType) const;
	virtual bool hasProcessedDataEnabled(XsDataIdentifier dataType) const;
	virtual XsString productCode() const;
	virtual XsString portName() const;
	virtual XsPortInfo portInfo() const;
	virtual XsBaudRate baudRate() const;
	virtual XsBaudRate serialBaudRate() const;
	virtual XsVersion hardwareVersion() const;
#ifndef XSENS_NO_PORT_NUMBERS
	virtual int XSNOLINUXEXPORT portNumber() const;
#endif
	virtual bool startRecording();
	virtual bool triggerStartRecording();
	virtual bool stopRecording();
	int64_t getStartRecordingPacketId() const;
	int64_t getStopRecordingPacketId() const;

	virtual void setOptions(XsOption enable, XsOption disable);
	virtual bool areOptionsEnabled(XsOption options) const;
	XsOption getOptions() const;

	virtual bool sendCustomMessage(const XsMessage& messageSend, bool waitForResult, XsMessage& messageReceive, int timeout = 0);
	virtual bool sendRawMessage(const XsMessage& message);

	XSNOEXPORT virtual bool sendCustomMessage(const XsMessage& messageSend, bool waitForResult, XsXbusMessageId messageId, XsMessage& messageReceive, int timeout = 0);
	XSNOEXPORT virtual bool waitForCustomMessage(XsXbusMessageId messageId, XsMessage& messageReceive, int timeout = 0);
	XSNOEXPORT virtual bool waitForCustomMessage(std::shared_ptr<ReplyObject> reply, XsMessage &messageReceive, int timeout);
	virtual std::shared_ptr<ReplyObject> addReplyObject(XsXbusMessageId messageId, uint8_t data);

	XSNOEXPORT virtual void handleMessage(const XsMessage &msg);
	XSNOEXPORT virtual void handleDataPacket(const XsDataPacket& packet);
	XSNOEXPORT virtual void handleNonDataMessage(const XsMessage &msg);
	XSNOEXPORT virtual void handleErrorMessage(const XsMessage &msg);
	XSNOEXPORT virtual void handleWarningMessage(const XsMessage &msg);

	virtual bool setSerialBaudRate(XsBaudRate baudrate);

	virtual XsIntArray portConfiguration() const;
	virtual bool setPortConfiguration(XsIntArray& config);

	virtual bool isMotionTracker() const;

	virtual XsOperationalMode operationalMode() const;
	virtual bool setOperationalMode(XsOperationalMode mode);

	virtual int updateRate() const;
	virtual bool setUpdateRate(int rate);

	virtual XsDeviceOptionFlag deviceOptionFlags() const;
	virtual bool setDeviceOptionFlags(XsDeviceOptionFlag setFlags, XsDeviceOptionFlag clearFlags);

	virtual XsOutputConfigurationArray outputConfiguration() const;
	virtual XsOutputConfigurationArray processedOutputConfiguration() const;
	virtual bool setOutputConfiguration(XsOutputConfigurationArray& config);
	virtual bool isInStringOutputMode() const;
	virtual XsCanOutputConfigurationArray canOutputConfiguration() const;
	virtual bool setCanOutputConfiguration(XsCanOutputConfigurationArray& config);
	virtual uint32_t canConfiguration() const;
	virtual bool setCanConfiguration(uint32_t config);

	virtual bool usesLegacyDeviceMode() const;

	virtual uint16_t stringOutputType() const;
	virtual uint16_t stringSamplePeriod() const;
	virtual uint16_t stringSkipFactor() const;

	virtual bool setStringOutputMode(uint16_t type, uint16_t period, uint16_t skipFactor);
	virtual XsStringOutputTypeArray supportedStringOutputTypes() const;

	virtual int dataLength() const;

	virtual XsSyncSettingArray syncSettings() const;
	virtual bool setSyncSettings(const XsSyncSettingArray& settingList);
	virtual bool isSyncMaster() const;
	virtual bool isSyncSlave() const;
	virtual XsSyncSettingArray supportedSyncSettings() const;
	static XsSyncSettingArray supportedSyncSettings(XsDeviceId deviceId);

	virtual bool gotoMeasurement();
	virtual bool gotoConfig();

	virtual bool restoreFactoryDefaults();

	virtual bool reset();
	XSNOEXPORT virtual bool reset(bool skipDeviceIdCheck);

	virtual bool reopenPort(bool gotoConfig, bool skipDeviceIdCheck = false);

	virtual void writeDeviceSettingsToFile();

	virtual void flushInputBuffers();

	virtual XsSyncRole syncRole() const;
	virtual bool loadLogFile();
	virtual bool abortLoadLogFile();
	virtual XsString logFileName() const;
	virtual bool resetOrientation(XsResetMethod resetmethod);
	virtual bool resetLogFileReadPosition();
	XsFilePos logFileSize() const;
	XsFilePos logFileReadPosition() const;
	virtual bool updateCachedDeviceInformation();
	virtual bool enableProtocol(XsProtocolType protocol);
	virtual bool disableProtocol(XsProtocolType protocol);
	virtual bool isProtocolEnabled(XsProtocolType protocol) const;

	virtual uint32_t deviceBufferSize();
	virtual bool setDeviceBufferSize(uint32_t frames);

	virtual XsConnectivityState connectivityState() const;
	virtual void waitForAllDevicesInitialized();

	// MtContainer
	virtual std::vector<XsDevice*> children() const;
	virtual int childCount() const;

	// Awinda Station
	virtual bool enableRadio(int channel);
	virtual bool disableRadio();
	virtual int radioChannel() const;
	virtual bool isRadioEnabled() const;
	virtual bool makeOperational();
	virtual bool isOperational() const;
	virtual bool isInSyncStationMode();
	virtual bool setSyncStationMode(bool enabled);

	virtual bool stealthMode() const;
	virtual bool setStealthMode(bool enabled);

	virtual void discardRetransmissions(int64_t firstNewPacketId);

	//virtual int radioQualityIndication() const;
	XSNOEXPORT virtual void handleMasterIndication(const XsMessage &message);
	virtual bool abortFlushing();
	virtual bool setDeviceAccepted(const XsDeviceId& deviceId);
	virtual bool setDeviceRejected(const XsDeviceId& deviceId);
	virtual bool setAccessControlMode(XsAccessControlMode mode, const XsDeviceIdArray& initialList);
	virtual XsAccessControlMode accessControlMode() const;
	virtual XsDeviceIdArray currentAccessControlList() const;

	virtual XsResultValue setDeviceParameter(XsDeviceParameter const& parameter);
	virtual XsResultValue deviceParameter(XsDeviceParameter& parameter) const;

	virtual XsGnssPlatform gnssPlatform() const;
	virtual bool setGnssPlatform(XsGnssPlatform gnssPlatform);

	// MTw
	virtual bool acceptConnection();
	virtual bool rejectConnection();
	virtual int wirelessPriority() const;
	virtual bool setWirelessPriority(int priority);
	virtual XsRejectReason rejectReason() const;

	virtual bool requestBatteryLevel();
	virtual XsTimeStamp batteryLevelTime();
	virtual bool setTransportMode(bool transportModeEnabled);
	virtual bool transportMode();
	virtual int16_t lastKnownRssi() const;
	XSNOEXPORT virtual void setPacketErrorRate(int per);
	virtual int packetErrorRate() const;

	virtual bool isBlueToothEnabled() const;
	virtual bool setBlueToothEnabled(bool enabled);
	virtual bool isBusPowerEnabled() const;
	virtual bool setBusPowerEnabled(bool enabled);
	virtual bool powerDown();
	virtual XsErrorMode errorMode() const;
	virtual bool setErrorMode(XsErrorMode errormode);

	// MT device
	virtual bool setHeadingOffset(double offset);
	virtual double headingOffset() const;
	virtual bool setLocationId(int id);
	virtual int locationId() const;
	virtual XsDevice* getDeviceFromLocationId(uint16_t locId);
	virtual XsMatrix objectAlignment() const;
	virtual bool setObjectAlignment(const XsMatrix &matrix);
	virtual double gravityMagnitude() const;
	virtual bool setGravityMagnitude(double mag);
	virtual XsVector initialPositionLLA() const;
	virtual bool setInitialPositionLLA(const XsVector& lla);
	virtual XsTimeInfo utcTime() const;
	virtual bool setUtcTime(const XsTimeInfo& time);
	virtual bool reinitialize();

	virtual XsFilterProfile xdaFilterProfile() const;
	virtual bool setXdaFilterProfile(int profileType);
	virtual bool setXdaFilterProfile(XsString const& profileType);
	virtual XsFilterProfile onboardFilterProfile() const;
	virtual bool setOnboardFilterProfile(int profileType);
	virtual bool setOnboardFilterProfile(XsString const& profileType);
	virtual bool replaceFilterProfile(XsFilterProfile const& profileCurrent, XsFilterProfile const& profileNew);
	virtual XsFilterProfileArray availableOnboardFilterProfiles() const;
	virtual XsFilterProfileArray availableXdaFilterProfiles() const;
	virtual double accelerometerRange() const;
	virtual double gyroscopeRange() const;
	virtual bool setNoRotation(uint16_t duration);
	virtual bool startRepresentativeMotion();
	virtual bool representativeMotionState();
	virtual XsIccRepMotionResult stopRepresentativeMotion();
	virtual bool storeIccResults();
	virtual uint16_t rs485TransmissionDelay() const;
	virtual bool setRs485TransmissionDelay(uint16_t delay);
	virtual XsSelfTestResult runSelfTest();
	virtual bool requestData();
	virtual bool storeFilterState();

	virtual XsDataPacket getDataPacketByIndex(XsSize index) const;
	XsSize getDataPacketCount() const;
	XsDataPacket lastAvailableLiveData() const;
	XsDataPacket takeFirstDataPacketInQueue();

	// MTix device
	virtual bool isInitialBiasUpdateEnabled() const;
	virtual bool setInitialBiasUpdateEnabled(bool enable);
	virtual bool isFixedGravityEnabled() const;
	virtual bool setFixedGravityEnabled(bool enable);

	virtual XsResultValue createConfigFile(const XsString& filename);
	virtual XsResultValue applyConfigFile(const XsString& filename);

	// MtMk4 device
	virtual bool setAlignmentRotationMatrix(XsAlignmentFrame frame, const XsMatrix& matrix);
	virtual XsMatrix alignmentRotationMatrix(XsAlignmentFrame frame) const;
	virtual bool setAlignmentRotationQuaternion(XsAlignmentFrame frame, const XsQuaternion& quat);
	virtual XsQuaternion alignmentRotationQuaternion(XsAlignmentFrame frame) const;

	//! \returns The device mutex.
	xsens::GuardedMutex* mutex() const { return &m_deviceMutex; }

	virtual bool deviceIsDocked(XsDevice *dev) const;

	bool isLoadLogFileInProgress() const;
	void waitForLoadLogFileDone() const;

	XSNOEXPORT virtual bool messageLooksSane(const XsMessage &msg) const;
	XSNOEXPORT virtual void prepareForTermination();

	// MTi with GPS/GNSS support
	XSNOEXPORT virtual bool setLeverArm(const XsVector &arm);
	XSNOEXPORT virtual XsVector leverArm() const;
	XSNOEXPORT virtual bool requestUtcTime();

	// Snapshot related
	XSNOEXPORT virtual void handleUnavailableData(int64_t frameNumber);

	XSNOEXPORT virtual bool writeEmtsPage(uint8_t const* data, int pageNr, int bankNr);
	XSNOEXPORT void setSkipEmtsReadOnInit(bool skip);

	XSNOEXPORT virtual bool readEmtsAndDeviceConfiguration();

	virtual uint32_t supportedStatusFlags() const;

protected:
	virtual XsDevice const* firstChild() const;
	virtual void setRecordingStartFrame(uint16_t startFrame);
	virtual void setRecordingStopFrame(uint16_t stopFrame);

	/*! \brief Uses log interface for a given data logger
		\param logger The data logger
		\details For testing purposes
	*/
	void useLogInterface(DataLogger* logger);

	friend struct XsDeviceEx;
	/*! \private @{ */
	void setFirmwareVersion(const XsVersion &version);
	void extractFirmwareVersion(XsMessage const& message);

	friend class MtContainer;
	virtual void setDeviceState(XsDeviceState state);
	virtual void updateDeviceState(XsDeviceState state);

	void removeIfNoRefs();

	virtual bool scheduleOrientationReset(XsResetMethod method);

	explicit XsDevice(XsDeviceId id);
	explicit XsDevice(Communicator* comm);
	explicit XsDevice(MtContainer *master, const XsDeviceId &childDeviceId);

	inline const XsDeviceConfiguration& deviceConfig() const
	{ return m_config; }

	void setDeviceId(const XsDeviceId &deviceId);

	XsOutputConfiguration findConfiguration(XsDataIdentifier dataType) const;

	virtual void writeMessageToLogFile(const XsMessage &message);

	virtual void writeFilterStateToFile();

	virtual void processLivePacket(XsDataPacket& pack);
	virtual void processBufferedPacket(XsDataPacket& pack);

	bool readDeviceConfiguration();

	inline XsDataPacket& latestLivePacket()
	{
		assert(m_latestLivePacket); assert(m_deviceMutex.haveGuardedLock());
		return *m_latestLivePacket;
	}
	inline XsDataPacket& latestBufferedPacket()
	{
		assert(m_latestBufferedPacket); assert(m_deviceMutex.haveGuardedLock());
		return *m_latestBufferedPacket;
	}
	inline XsDataPacket const& latestLivePacketConst() const
	{
		assert(m_latestLivePacket); assert(m_deviceMutex.haveGuardedLock());
		return *m_latestLivePacket;
	}
	inline XsDataPacket const& latestBufferedPacketConst() const
	{
		assert(m_latestBufferedPacket); assert(m_deviceMutex.haveGuardedLock());
		return *m_latestBufferedPacket;
	}
	virtual int64_t latestLivePacketId() const;
	virtual int64_t latestBufferedPacketId() const;

	virtual void resetPacketStamping();

	void updateConnectivityState(XsConnectivityState newState);
	virtual XsConnectivityState defaultChildConnectivityState() const;

	void setInitialized(bool initialized) { m_isInitialized = initialized; }
	void setTerminationPrepared(bool prepared) { m_terminationPrepared  = prepared; }

	virtual bool shouldWriteMessageToLogFile(const XsMessage &msg) const;
	virtual bool shouldWriteMessageToLogFile(const XsDevice *dev, const XsMessage &message) const;

public:
	XSNOEXPORT virtual void onMessageSent(const XsMessage &message);
	XSNOEXPORT virtual void onMessageReceived(const XsMessage &message);
	XSNOEXPORT virtual void onMessageDetected(XsProtocolType type, const XsByteArray &rawMessage);

	XSNOEXPORT virtual void onSessionRestarted();
	XSNOEXPORT virtual void onConnectionLost();
	XSNOEXPORT virtual void onEofReached();
	XSNOEXPORT virtual void onWirelessConnectionLost();
	XSNOEXPORT virtual int64_t deviceRecordingBufferItemCount(int64_t& lastCompletePacketId) const;

protected:
	bool doTransaction(const XsMessage &snd) const;
	bool doTransaction(const XsMessage &snd, XsMessage &rcv) const;
	bool doTransaction(const XsMessage &snd, XsMessage &rcv, uint32_t timeout) const;
	bool doTransaction(const XsMessage &snd, uint32_t timeout) const;

	bool justWriteSetting() const {return m_justWriteSetting;}

	virtual void clearProcessors();
	virtual void clearDataCache();
	virtual void insertIntoDataCache(int64_t pid, XsDataPacket* pack);
	virtual void reinitializeProcessors();
	virtual bool expectingRetransmissionForPacket(int64_t packetId) const;

	virtual bool resetRemovesPort() const;

	virtual inline bool isSoftwareFilteringEnabled() const { return false; }
	virtual inline bool isSoftwareCalibrationEnabled() const { return false; }


	virtual void setStartRecordingPacketId(int64_t startFrame);
	virtual void setStopRecordingPacketId(int64_t stopFrame);
	virtual void endRecordingStream();
	virtual void clearCacheToRecordingStart();

	/*! @} */

	//! A device mutex.
	mutable xsens::GuardedMutex m_deviceMutex;

	//! \brief A copy of the latest ready live packet. This is the packet with the highest 64-bit sample counter so far. Use latestLivePacket() to access.
	XsDataPacket* m_latestLivePacket;

	//! \brief A copy of the latest ready recording packet. This is the last packet that was popped off the front of m_dataCache. Use latestBufferedPacket() to access.
	XsDataPacket* m_latestBufferedPacket;

	void setCommunicator(Communicator* comm);

	//! \returns True if skip EMTS read on initialization is set to true
	bool skipEmtsReadOnInit() const { return m_skipEmtsReadOnInit; }
	static bool checkDataEnabled(XsDataIdentifier dataType, XsOutputConfigurationArray const & configurations);
	virtual bool shouldDataMsgBeRecorded(const XsMessage &msg) const;
	virtual bool shouldDoRecordedCallback(XsDataPacket const& packet) const;
	virtual bool interpolateMissingData(XsDataPacket const & pack, XsDataPacket const & prev, std::function<void (XsDataPacket*)> packetHandler);

	//! \brief A data cache
	DataPacketCache m_dataCache;

	//! \brief A packet ID of the last sample we know to be unavailable
	int64_t m_unavailableDataBoundary;

	//! \brief An ID of the device
	XsDeviceId m_deviceId;

	//! \brief The last result of an operation
	mutable LastResultManager m_lastResult;

	//! \brief A current device state
	XsDeviceState m_state;

	//! \brief A current device connectivity state
	XsConnectivityState m_connectivity;

	//! \brief A time stamp for an OK last data
	XsTimeStamp m_lastDataOkStamp;

	//! \brief A device configuration
	XsDeviceConfiguration m_config;

	//! \brief A firmware version
	XsVersion m_firmwareVersion;

	//! \brief A communicator
	Communicator* m_communicator;

	//! \brief A data logger for a file interface
	DataLogger* m_logFileInterface;

	//! \brief A device object
	XsDevice* m_master;

	//! \brief A reference counter
	volatile std::atomic_int m_refCounter;

	//! \brief Write to file boolean variable
	bool m_writeToFile;

	//! \brief Is intialized boolean variable
	bool m_isInitialized;

	//! \brief Termination prepared boolean variable
	bool m_terminationPrepared;

	//! \brief Go to confing on close boolean variable
	bool m_gotoConfigOnClose;

	//! \brief Just write setting boolean variable
	bool m_justWriteSetting;

	/*! \brief Skip EMTS read on init boolean variable
		\details Required for the firmware updater to retain EMTS while rebooting devices
	*/
	bool m_skipEmtsReadOnInit;

	//! \brief A packet stamper
	PacketStamper m_packetStamper;

	//! \brief The options
	XsOption m_options;

	/*! \brief An EMTS blob from device.
		\note Used in public source for storing EMTS for log file header.
	*/
	XsMessage m_emtsBlob;

protected:
	//These members are protected to give direct access to tests

	//! \brief The ID of the first packet that should be / was recorded
	int64_t m_startRecordingPacketId;

	//! \brief The ID of the last packet that should be / was recorded. Only valid in Recording/Flushing states
	int64_t m_stopRecordingPacketId;

	//! \brief The ID of the last packet that was recorded. Remains valid after Flushing has ended, until a new recording is started.
	int64_t m_stoppedRecordingPacketId;

	virtual void clearExternalPacketCaches();
	void updateLastAvailableLiveDataCache(XsDataPacket const& pack);
	void retainPacket(XsDataPacket const& pack);
	static bool packetContainsRetransmission(XsDataPacket const& pack);

	//! \brief A linear data packet cache
	std::deque<XsDataPacket*> m_linearPacketCache;

	//! \brief A last available live data cache
	XsDataPacket* m_lastAvailableLiveDataCache;

	/*! \brief To a dump file.
		\details For debugging purposes only, but doesn't do any harm to always be there.
	*/
	DebugFileType* m_toaDumpFile;

	XSENS_DISABLE_COPY(XsDevice)
};

#ifndef XDA_PRIVATE_BUILD
#include "xsdevice_public.h"
#else
#include "xsdeviceex.h"
#endif

#endif
