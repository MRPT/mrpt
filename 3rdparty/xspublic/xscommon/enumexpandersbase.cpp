
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

#include "enumexpandersbase.h"

extern Journaller* gJournal;

JLENUMEXPANDER(XsResultValue,
	JLENUMCASE(XRV_OK) //!< 0: Operation was performed successfully

	// communication protocol
	JLENUMCASE(XRV_NOBUS) //!< 1: No bus communication possible
	JLENUMCASE(XRV_BUSNOTREADY) //!< 2: InitBus and/or SetBID are not issued
	JLENUMCASE(XRV_INVALIDPERIOD) ///!< 3: Period sent is invalid
	JLENUMCASE(XRV_INVALIDMSG) //!< 4: The message is invalid or not implemented
	JLENUMCASE(XRV_INITBUSFAIL1) //!< 16: A slave did not respond to WaitForSetBID
	JLENUMCASE(XRV_INITBUSFAIL2) //!< 17: An incorrect answer received after WaitForSetBID
	JLENUMCASE(XRV_INITBUSFAIL3) //!< 18: After four bus-scans still undetected Motion Trackers
	JLENUMCASE(XRV_SETBIDFAIL1) //!< 20: No reply to SetBID message during SetBID procedure
	JLENUMCASE(XRV_SETBIDFAIL2) //!< 21: Other than SetBIDAck received
	JLENUMCASE(XRV_MEASUREMENTFAIL1) //!< 24: Timer overflow - period too short to collect all data from Motion Trackers
	JLENUMCASE(XRV_MEASUREMENTFAIL2) //!< 25: Motion Tracker responds with other than SlaveData message
	JLENUMCASE(XRV_MEASUREMENTFAIL3) //!< 26: Total bytes of data of Motion Trackers including sample counter exceeds 255 bytes
	JLENUMCASE(XRV_MEASUREMENTFAIL4) //!< 27: Timer overflows during measurement
	JLENUMCASE(XRV_MEASUREMENTFAIL5) //!< 28: Timer overflows during measurement
	JLENUMCASE(XRV_MEASUREMENTFAIL6) //!< 29: No correct response from Motion Tracker during measurement
	JLENUMCASE(XRV_TIMEROVERFLOW) //!< 30: Timer overflows during measurement
	JLENUMCASE(XRV_BAUDRATEINVALID) //!< 32: Baud rate does not comply with valid range
	JLENUMCASE(XRV_INVALIDPARAM) //!< 33: An invalid parameter is supplied
	JLENUMCASE(XRV_MEASUREMENTFAIL7) //!< 35: TX PC Buffer is full
	JLENUMCASE(XRV_MEASUREMENTFAIL8) //!< 36: TX PC Buffer overflowJLENUMCASE( cannot fit full message
	JLENUMCASE(XRV_WIRELESSFAIL) //!< 37: Wireless subsystem failed
	JLENUMCASE(XRV_DEVICEERROR) //!< 40: The device generated an errorJLENUMCASE( try updating the firmware
	JLENUMCASE(XRV_DATAOVERFLOW) //!< 41: The device generates more data than the bus communication can handle (baud rate may be too low)
	JLENUMCASE(XRV_BUFFEROVERFLOW) //!< 42: The sample buffer of the device was full during a communication outage
	JLENUMCASE(XRV_EXTTRIGGERERROR) //!< 43: The external trigger is not behaving as configured
	JLENUMCASE(XRV_SAMPLESTREAMERROR) //!< 44: The sample stream detected an error in the ordering of sample data
	JLENUMCASE(XRV_POWER_DIP) //!< 45: A dip in the power supply was detected and recovered from
	JLENUMCASE(XRV_POWER_OVERCURRENT) //!< 46: A current limiter has been activatedJLENUMCASE( shutting down the device
	JLENUMCASE(XRV_OVERHEATING) //!< 47: Device temperature is not within operational limits
	JLENUMCASE(XRV_BATTERYLOW) //!< 48: Battery level reached lower limit
	JLENUMCASE(XRV_INVALIDFILTERPROFILE) //!< 49: Specified filter profile ID is not available on the device or the user is trying to duplicate an existing filter profile type
	JLENUMCASE(XRV_INVALIDSTOREDSETTINGS) //!< 50: The settings stored in the device's non volatile memory are invalid
	JLENUMCASE(XRV_ACCESSDENIED) //!< 51: Request for control of the device was denied
	JLENUMCASE(XRV_FILEERROR) //!< 52: Failure reading, writing, opening or closing a file
	JLENUMCASE(XRV_OUTPUTCONFIGERROR) //!< 53: Erroneous output configuration, device can not go to measurement

	// CMT / XDA / XME / etc
	JLENUMCASE(XRV_ERROR) //!< 256: A generic error occurred
	JLENUMCASE(XRV_NOTIMPLEMENTED) //!< 257: Operation not implemented in this version (yet)
	JLENUMCASE(XRV_TIMEOUT) //!< 258: A timeout occurred
	JLENUMCASE(XRV_TIMEOUTNODATA) //!< 259: Operation aborted because of no data read
	JLENUMCASE(XRV_CHECKSUMFAULT) //!< 260: Checksum fault occurred
	JLENUMCASE(XRV_OUTOFMEMORY) //!< 261: No internal memory available
	JLENUMCASE(XRV_NOTFOUND) //!< 262: The requested item was not found
	JLENUMCASE(XRV_UNEXPECTEDMSG) //!< 263: Unexpected message received (e.g. no acknowledge message received)
	JLENUMCASE(XRV_INVALIDID) //!< 264: Invalid id supplied
	JLENUMCASE(XRV_INVALIDOPERATION) //!< 265: Operation is invalid at this point
	JLENUMCASE(XRV_INSUFFICIENTSPACE) //!< 266: Insufficient buffer space available
	JLENUMCASE(XRV_INPUTCANNOTBEOPENED) //!< 267: The specified i/o device can not be opened
	JLENUMCASE(XRV_OUTPUTCANNOTBEOPENED) //!< 268: The specified i/o device can not be opened
	JLENUMCASE(XRV_ALREADYOPEN) //!< 269: An I/O device is already opened with this object
	JLENUMCASE(XRV_ENDOFFILE) //!< 270: End of file is reached
	JLENUMCASE(XRV_COULDNOTREADSETTINGS) //!< 271: A required settings file could not be opened or is missing some data
	JLENUMCASE(XRV_NODATA) //!< 272: No data is available
	JLENUMCASE(XRV_READONLY) //!< 273: Tried to change a read-only value
	JLENUMCASE(XRV_NULLPTR) //!< 274: Tried to supply a NULL value where it is not allowed
	JLENUMCASE(XRV_INSUFFICIENTDATA) //!< 275: Insufficient data was supplied to a function
	JLENUMCASE(XRV_BUSY) //!< 276: Busy processingJLENUMCASE( try again later
	JLENUMCASE(XRV_INVALIDINSTANCE) //!< 277: Invalid instance calledJLENUMCASE( because of an invalid or missing license
	JLENUMCASE(XRV_DATACORRUPT) //!< 278: A trusted data stream proves to contain corrupted data

	JLENUMCASE(XRV_READINITFAILED) //!< 279: Failure during read of settings
	JLENUMCASE(XRV_NOXMFOUND) //!< 280: Could not find any MVN-compatible hardware
	JLENUMCASE(XRV_DEVICECOUNTZERO) //!< 282: No xsens devices found
	JLENUMCASE(XRV_MTLOCATIONINVALID) //!< 283: One or more sensors are not where they were expected
	JLENUMCASE(XRV_INSUFFICIENTMTS) //!< 284: Not enough sensors were found
	JLENUMCASE(XRV_INITFUSIONFAILED) //!< 285: Failure during initialization of Fusion Engine
	JLENUMCASE(XRV_OTHER) //!< 286: Something else was received than was requested

	JLENUMCASE(XRV_NOFILEOPEN) //!< 287: No file opened for reading/writing
	JLENUMCASE(XRV_NOPORTOPEN) //!< 288: No serial port opened for reading/writing
	JLENUMCASE(XRV_NOFILEORPORTOPEN) //!< 289: No file or serial port opened for reading/writing
	JLENUMCASE(XRV_PORTNOTFOUND) //!< 290: A required port could not be found
	JLENUMCASE(XRV_INITPORTFAILED) //!< 291: The low-level port handler failed to initialize
	JLENUMCASE(XRV_CALIBRATIONFAILED) //!< 292: A calibration routine failed

	JLENUMCASE(XRV_CONFIGCHECKFAIL) //!< 293: The in-config check of the device failed
	JLENUMCASE(XRV_ALREADYDONE) //!< 294: The operation is once only and has already been performed

	JLENUMCASE(XRV_SYNC_SINGLE_SLAVE) //!< 295: The single connected device is configured as a slave
	JLENUMCASE(XRV_SYNC_SECOND_MASTER) //!< 296: More than one master was detected
	JLENUMCASE(XRV_SYNC_NO_SYNC) //!< 297: A device was detected that was neither master nor slave
	JLENUMCASE(XRV_SYNC_NO_MASTER) //!< 298: No master detected
	JLENUMCASE(XRV_SYNC_DATA_MISSING) //!< 299: A device is not sending enough data

	JLENUMCASE(XRV_VERSION_TOO_LOW) //!< 300: The version of the object is too low for the requested operation
	JLENUMCASE(XRV_VERSION_PROBLEM) //!< 301: The object has an unrecognised versionJLENUMCASE( so it's not safe to perform the operation

	JLENUMCASE(XRV_ABORTED) //!< 302: The process was aborted by an external eventJLENUMCASE( usually a user action or process termination
	JLENUMCASE(XRV_UNSUPPORTED) //!< 303: The requested functionality is not supported by the device

	JLENUMCASE(XRV_PACKETCOUNTERMISSED) //!< 304: A packet counter value was missed

	JLENUMCASE(XRV_MEASUREMENTFAILED) //!< 305: An error occurred while trying to put the device in measurement mode
	JLENUMCASE(XRV_STARTRECORDINGFAILED) //!< 306: A device could not start recording
	JLENUMCASE(XRV_STOPRECORDINGFAILED) //!< 307: A device could not stop recording

	JLENUMCASE(XRV_RADIO_CHANNEL_IN_USE) //!< 311: Radio channel is in use by another system
	JLENUMCASE(XRV_UNEXPECTED_DISCONNECT) //!< 312: Motion tracker disconnected unexpectedly
	JLENUMCASE(XRV_TOO_MANY_CONNECTED_TRACKERS) //!< 313: Too many motion trackers connected
	JLENUMCASE(XRV_GOTOCONFIGFAILED) //!< 314: A device could not be put in config mode
	JLENUMCASE(XRV_OUTOFRANGE) //!< 315: Device has gone out of range
	JLENUMCASE(XRV_BACKINRANGE) //!< 316: Device is back in rangeJLENUMCASE( resuming normal operation
	JLENUMCASE(XRV_EXPECTED_DISCONNECT) //!< 317: The device was disconnected
	JLENUMCASE(XRV_RESTORE_COMMUNICATION_FAILED) //!< 318: Restore communication failed
	JLENUMCASE(XRV_RESTORE_COMMUNICATION_STOPPED) //!< 319: Restore communication was stopped
	JLENUMCASE(XRV_EXPECTED_CONNECT) //!< 320: The device was connected

	// notifications
	JLENUMCASE(XRV_SHUTTINGDOWN) //!< 400: The device is shutting down
	JLENUMCASE(XRV_GNSSCONFIGURATIONERROR)	//!< 401: A configuration item was refused by the GNSS module
	JLENUMCASE(XRV_GNSSCOMMTIMEOUT) //!< 402: The communication with the GNSS module timed out
	JLENUMCASE(XRV_GNSSERROR) //!< 403: Communication between the device and the GNSS module failed
	JLENUMCASE(XRV_DEVICE_NOT_CALIBRATED)	//!< 404: The EMTS of the device does not contain calibration data
)

JLENUMEXPANDERHEX(XsXbusMessageId,
	//JLENUMCASE(XMID_InvalidMessage) // 0x00,
	JLENUMCASE(XMID_ReqDid) // 0x00,
	JLENUMCASE(XMID_DeviceId) // 0x01,
	JLENUMCASE(XMID_Initbus) // 0x02,
	JLENUMCASE(XMID_InitBusResults) // 0x03,
	JLENUMCASE(XMID_ReqPeriod) // 0x04,
	JLENUMCASE(XMID_ReqPeriodAck) // 0x05,
	//JLENUMCASE(XMID_SetPeriod) // 0x04,
	//JLENUMCASE(XMID_SetPeriodAck) // 0x05,
	JLENUMCASE2(XMID_SetBid, "XMID_SetBid/XMID_AutoStart") // 0x06,
	JLENUMCASE2(XMID_SetBidAck, "XMID_SetBidAck/XMID_AutoStartAck") // 0x07,
	//JLENUMCASE(XMID_AutoStart) // 0x06,
	//JLENUMCASE(XMID_AutoStartAck) // 0x07,
	JLENUMCASE(XMID_BusPower) // 0x08,
	JLENUMCASE(XMID_BusPowerAck) // 0x09,
	JLENUMCASE(XMID_ReqDataLength) // 0x0A,
	JLENUMCASE(XMID_DataLength) // 0x0B,
	JLENUMCASE(XMID_ReqConfiguration) // 0x0C,
	JLENUMCASE(XMID_Configuration) // 0x0D,
	JLENUMCASE(XMID_RestoreFactoryDef) // 0x0E,
	JLENUMCASE(XMID_RestoreFactoryDefAck) // 0x0F,
	JLENUMCASE(XMID_GotoMeasurement) // 0x10,
	JLENUMCASE(XMID_GotoMeasurementAck) // 0x11,
	JLENUMCASE(XMID_ReqFirmwareRevision) // 0x12,
	JLENUMCASE(XMID_FirmwareRevision) // 0x13,
	JLENUMCASE(XMID_ReqUniqueId) // 0x14,
	JLENUMCASE(XMID_UniqueId) // 0x15,
	JLENUMCASE(XMID_ReqBodypackMode) // 0x16,
	JLENUMCASE(XMID_ReqBodypackAck) // 0x17,
	//JLENUMCASE(XMID_SetXmOutputMode) // 0x16,
	//JLENUMCASE(XMID_SetXmOutputModeAck) // 0x17,
	JLENUMCASE(XMID_ReqBaudrate) // 0x18,
	JLENUMCASE(XMID_ReqBaudrateAck) // 0x19,
	//JLENUMCASE(XMID_SetBaudrate) // 0x18,
	//JLENUMCASE(XMID_SetBaudrateAck) // 0x19,
	//JLENUMCASE(XMID_ReqSyncMode) // 0x1A,
	//JLENUMCASE(XMID_ReqSyncModeAck) // 0x1B,
	//JLENUMCASE(XMID_SetSyncMode) // 0x1A,
	//JLENUMCASE(XMID_SetSyncModeAck) // 0x1B,
	JLENUMCASE(XMID_ReqProductCode) // 0x1C,
	JLENUMCASE(XMID_ProductCode) // 0x1D,
	JLENUMCASE(XMID_ReqProcessingFlags) // 0x20,
	JLENUMCASE(XMID_ReqProcessingFlagsAck) // 0x21,
	//JLENUMCASE(XMID_SetProcessingFlags) // 0x20,
	//JLENUMCASE(XMID_SetProcessingFlagsAck) // 0x21,
	JLENUMCASE(XMID_SetNoRotation) // 0x22,
	JLENUMCASE(XMID_SetNoRotationAck) // 0x23,
	JLENUMCASE(XMID_RunSelfTest) // 0x24,
	JLENUMCASE(XMID_SelfTestResults) // 0x25,
	JLENUMCASE(XMID_GotoConfig) // 0x30,
	JLENUMCASE(XMID_GotoConfigAck) // 0x31,
	JLENUMCASE2(XMID_BusData, "XMID_BusData/XMID_MtData/XMID_PrepareData") // 0x32,
	//JLENUMCASE(XMID_MtData) // 0x32,
	JLENUMCASE(XMID_ReqInputTrigger) // 0x26,
	JLENUMCASE(XMID_ReqInputTriggerAck) // 0x27,
	//JLENUMCASE(XMID_SetInputTrigger) // 0x26,
	//JLENUMCASE(XMID_SetInputTriggerAck) // 0x27,
	JLENUMCASE(XMID_ReqOutputTrigger) // 0x28,
	JLENUMCASE(XMID_ReqOutputTriggerAck) // 0x29,
	//JLENUMCASE(XMID_SetOutputTrigger) // 0x28,
	//JLENUMCASE(XMID_SetSyncStationMode) // 0x2A,
	//JLENUMCASE(XMID_SetSyncStationModeAck) // 0x2B,
	JLENUMCASE(XMID_ReqSyncStationMode) // 0x2A,
	JLENUMCASE(XMID_ReqSyncStationModeAck) // 0x2B,
	//JLENUMCASE(XMID_SetSyncBoxMode) // 0x2A,
	//JLENUMCASE(XMID_SetSyncBoxModeAck) // 0x2B,
	//JLENUMCASE(XMID_ReqSyncBoxMode) // 0x2A,
	//JLENUMCASE(XMID_ReqSyncBoxModeAck) // 0x2B,
	//JLENUMCASE(XMID_SetSyncConfiguration) // 0x2C,
	JLENUMCASE(XMID_SetSyncConfigurationAck) // 0x2D,
	JLENUMCASE(XMID_ReqSyncConfiguration) // 0x2C,
	//JLENUMCASE(XMID_SyncConfiguration) // 0x2D,
	JLENUMCASE(XMID_DriverDisconnect) // 0x2E,
	JLENUMCASE(XMID_DriverDisconnectAck) // 0x2F,
	//JLENUMCASE(XMID_PrepareData) // 0x32,
	JLENUMCASE(XMID_ReqData) // 0x34,
	JLENUMCASE(XMID_ReqDataAck) // 0x35,
	JLENUMCASE(XMID_MtData2) // 0x36,
	JLENUMCASE(XMID_MtData2Ack) // 0x37,
	JLENUMCASE(XMID_RequestControl) // 0x38,
	JLENUMCASE(XMID_RequestControlAck) // 0x39,
	JLENUMCASE(XMID_SetDataPort) // 0x3A,
	JLENUMCASE(XMID_SetDataPortAck) // 0x3B,
	JLENUMCASE(XMID_ReqRetransmission) // 0x3C,
	JLENUMCASE(XMID_ReqRetransmissionAck) // 0x3D,
	JLENUMCASE(XMID_Wakeup) // 0x3E,
	JLENUMCASE(XMID_WakeupAck) // 0x3F,
	JLENUMCASE(XMID_Reset) // 0x40,
	JLENUMCASE(XMID_ResetAck) // 0x41,
	JLENUMCASE(XMID_Error) // 0x42,
	JLENUMCASE(XMID_XmPowerOff) // 0x44,
	JLENUMCASE(XMID_MasterIndication) // 0x46,
	JLENUMCASE(XMID_ReqOptionFlags) // 0x48,
	JLENUMCASE(XMID_ReqOptionFlagsAck) // 0x49,
	//JLENUMCASE(XMID_SetOptionFlags) // 0x48,
	//JLENUMCASE(XMID_SetOptionFlagsAck) // 0x49,
	JLENUMCASE(XMID_ReqStealthMode) // 0x4A,
	JLENUMCASE(XMID_StealthMode) // 0x4B,
	//JLENUMCASE(XMID_SetStealthMode) // 0x4A,
	//JLENUMCASE(XMID_SetStealthModeAck) // 0x4B,
	JLENUMCASE(XMID_UserInterface) // 0x4C,
	JLENUMCASE(XMID_UserInterfaceAck) // 0x4D,
	JLENUMCASE(XMID_EndOfRecording) // 0x4E,
	JLENUMCASE(XMID_EndOfRecordingAck) // 0x4F,
	JLENUMCASE(XMID_GotoTransparentMode) // 0x50,
	JLENUMCASE(XMID_GotoTransparentModeAck) // 0x51,
	JLENUMCASE(XMID_RunFactoryTest) // 0x56,
	JLENUMCASE(XMID_FactoryTestResults) // 0x57,
	JLENUMCASE(XMID_FactoryTestConnect) // 0x58,
	JLENUMCASE(XMID_FactoryTestConnectAck) // 0x59,
	JLENUMCASE(XMID_SetDataOutputDelay) // 0x5A,
	JLENUMCASE(XMID_SetDataOutputDelayAck) // 0x5B,
	JLENUMCASE(XMID_SetBodypackConfigFile) // 0x5C,
	JLENUMCASE(XMID_SetBodypackConfigFileAck)	// 0x5D,
	JLENUMCASE(XMID_ReqObrStatus) // 0x5E,
	JLENUMCASE(XMID_ObrStatus) // 0x5F,

	//JLENUMCASE(XMID_SetUtcTime) // 0x60,
	JLENUMCASE2(XMID_ReqUtcTime, "XMID_ReqUtcTime/XMID_FactoryTestSensorTiming") // 0x60,
	JLENUMCASE2(XMID_SetUtcTimeAck, "XMID_ReqUtcTimeAck/XMID_FactoryTestSensorTimingResults") // 0x61,
	//JLENUMCASE(XMID_UtcTime) // 0x61,
	//JLENUMCASE(XMID_FactoryTestSensorTiming) // 0x60,
	//JLENUMCASE(XMID_FactoryTestSensorTimingResults) // 0x61,
	JLENUMCASE(XMID_ReqAvailableFilterProfiles) // 0x62,
	JLENUMCASE(XMID_AvailableFilterProfiles) // 0x63,
	JLENUMCASE(XMID_ReqFilterProfile) // 0x64,
	JLENUMCASE(XMID_ReqFilterProfileAck) // 0x65,
	//JLENUMCASE(XMID_SetFilterProfile) // 0x64,
	//JLENUMCASE(XMID_SetFilterProfileAck) // 0x65,
	JLENUMCASE(XMID_ReqGravityMagnitude) // 0x66,
	JLENUMCASE(XMID_ReqGravityMagnitudeAck) // 0x67,
	//JLENUMCASE(XMID_SetGravityMagnitude) // 0x66,
	//JLENUMCASE(XMID_SetGravityMagnitudeAck) // 0x67,
	JLENUMCASE(XMID_ReqGnssLeverArm) // 0x68,
	JLENUMCASE(XMID_ReqGnssLeverArmAck) // 0x69,
	//JLENUMCASE(XMID_SetGnssLeverArm) // 0x68,
	//JLENUMCASE(XMID_SetGnssLeverArmAck) // 0x69,
	JLENUMCASE(XMID_ReqReplayMode) // 0x6C,
	JLENUMCASE(XMID_ReqReplayModeAck) // 0x6D,
	//JLENUMCASE(XMID_SetReplayMode) // 0x6C,
	//JLENUMCASE(XMID_SetReplayModeAck) // 0x6D,
	JLENUMCASE(XMID_ReqLatLonAlt) // 0x6E,
	JLENUMCASE(XMID_ReqLatLonAltAck) // 0x6F,
	//JLENUMCASE(XMID_SetLatLonAlt) // 0x6E,
	//JLENUMCASE(XMID_SetLatLonAltAck) // 0x6F,
	JLENUMCASE2(XMID_ReqXmErrorMode, "XMID_ReqXmErrorMode/XMID_ReqHeading") // 0x82,
	JLENUMCASE2(XMID_ReqXmErrorModeAck, "XMID_ReqXmErrorModeAck/XMID_ReqHeadingAck") // 0x83,
	//JLENUMCASE(XMID_SetXmErrorMode) // 0x82,
	//JLENUMCASE(XMID_SetXmErrorModeAck) // 0x83,
	JLENUMCASE2(XMID_ReqBufferSize, "XMID_ReqBufferSize/XMID_ReqLocationId") // 0x84,
	JLENUMCASE2(XMID_ReqBufferSizeAck, "XMID_ReqBufferSizeAck/XMID_ReqLocationIdAck") // 0x85,
	//JLENUMCASE(XMID_SetBufferSize) // 0x84,
	//JLENUMCASE(XMID_SetBufferSizeAck) // 0x85,
	//JLENUMCASE(XMID_ReqHeading) // 0x82,
	//JLENUMCASE(XMID_ReqHeadingAck) // 0x83,
	//JLENUMCASE(XMID_SetHeading) // 0x82,
	//JLENUMCASE(XMID_SetHeadingAck) // 0x83,
	JLENUMCASE(XMID_ReqMagneticField) // 0x6A,
	JLENUMCASE(XMID_ReqMagneticFieldAck) // 0x6B,
	//JLENUMCASE(XMID_SetMagneticField) // 0x6A,
	//JLENUMCASE(XMID_SetMagneticFieldAck) // 0x6B,
	JLENUMCASE(XMID_KeepAlive) // 0x70,
	JLENUMCASE(XMID_KeepAliveAck) // 0x71,
	JLENUMCASE(XMID_ReqConnectionSettings) // 0x78,
	JLENUMCASE(XMID_ReqConnectionSettingsAck) // 0x79,
	//JLENUMCASE(XMID_SetConnectionSettings) // 0x78,
	//JLENUMCASE(XMID_SetConnectionSettingsAck) // 0x79,
	JLENUMCASE(XMID_CloseConnection) // 0x72,
	JLENUMCASE(XMID_CloseConnectionAck) // 0x73,
	JLENUMCASE(XMID_IccCommand) // 0x74,
	JLENUMCASE(XMID_IccCommandAck) // 0x75,
	JLENUMCASE(XMID_ReqGnssPlatform) // 0x76,
	JLENUMCASE(XMID_ReqGnssPlatformAck) // 0x77,
	//JLENUMCASE(XMID_SetGnssPlatform) // 0x76,
	//JLENUMCASE(XMID_SetGnssPlatformAck) // 0x77,
	JLENUMCASE(XMID_BodyPackBundle) // 0x7A,
	JLENUMCASE(XMID_BodyPackBundleAck) // 0x7B,
	JLENUMCASE(XMID_ReqStationOptions) // 0x7C,
	JLENUMCASE(XMID_ReqStationOptionsAck) // 0x7D,
	//JLENUMCASE(XMID_ReqLocationId) // 0x84,
	//JLENUMCASE(XMID_ReqLocationIdAck) // 0x85,
	//JLENUMCASE(XMID_SetLocationId) // 0x84,
	//JLENUMCASE(XMID_SetLocationIdAck) // 0x85,
	JLENUMCASE(XMID_ReqExtOutputMode) // 0x86,
	JLENUMCASE(XMID_ReqExtOutputModeAck) // 0x87,
	//JLENUMCASE(XMID_SetExtOutputMode) // 0x86,
	//JLENUMCASE(XMID_SetExtOutputModeAck) // 0x87,
	JLENUMCASE(XMID_ReqStringOutputType) // 0x8E,
	JLENUMCASE(XMID_ReqStringOutputTypeAck) // 0x8F,
	//JLENUMCASE(XMID_SetStringOutputType) // 0x8E,
	//JLENUMCASE(XMID_SetStringOutputTypeAck) // 0x8F,
	JLENUMCASE2(XMID_ReqBatteryLevel, "XMID_ReqBatteryLevel/XMID_ReqInitTrackMode") // 0x88,
	JLENUMCASE2(XMID_Batterylevel, "XMID_Batterylevel/XMID_ReqInitTrackModeAck") // 0x89,
	//JLENUMCASE(XMID_ReqInitTrackMode) // 0x88,
	//JLENUMCASE(XMID_ReqInitTrackModeAck) // 0x89,
	//JLENUMCASE(XMID_SetInitTrackMode) // 0x88,
	//JLENUMCASE(XMID_SetInitTrackModeAck) // 0x89,
	JLENUMCASE2(XMID_ReqMasterSettings, "XMID_ReqMasterSettings/XMID_StoreFilterState") // 0x8A,
	JLENUMCASE2(XMID_MasterSettings, "XMID_MasterSettings/XMID_StoreFilterStateAck") // 0x8B,
	//JLENUMCASE(XMID_StoreFilterState) // 0x8A,
	//JLENUMCASE(XMID_StoreFilterStateAck) // 0x8B,
	JLENUMCASE(XMID_ReqPortConfig) // 0x8C,
	JLENUMCASE(XMID_PortConfig) // 0x8D,
	JLENUMCASE(XMID_ReqEmts) // 0x90,
	JLENUMCASE(XMID_EmtsData) // 0x91,
	JLENUMCASE(XMID_UpdateFilterProfile) // 0x92,
	JLENUMCASE(XMID_UpdateFilterProfileAck) // 0x93,
	JLENUMCASE(XMID_RestoreEmts) // 0x94,
	JLENUMCASE(XMID_RestoreEmtsAck) // 0x95,
	JLENUMCASE(XMID_StoreEmts) // 0x96,
	JLENUMCASE(XMID_StoreEmtsAck) // 0x97,
	JLENUMCASE(XMID_ClockSyncCommand) // 0x9A,
	JLENUMCASE(XMID_ClockSyncCommandAck) // 0x9B,
	JLENUMCASE(XMID_ReqActiveClockCorrection) // 0x9C,
	JLENUMCASE(XMID_ActiveClockCorrection) // 0x9D,
	JLENUMCASE(XMID_StoreActiveClockCorrection) // 0x9E,
	JLENUMCASE(XMID_StoreActiveClockCorrectionAck) // 0x9F,
	JLENUMCASE(XMID_ReqFilterSettings) // 0xA0,
	JLENUMCASE(XMID_ReqFilterSettingsAck) // 0xA1,
	//JLENUMCASE(XMID_SetFilterSettings) // 0xA0,
	//JLENUMCASE(XMID_SetFilterSettingsAck) // 0xA1,
	JLENUMCASE(XMID_ReqAmd) // 0xA2,
	JLENUMCASE(XMID_ReqAmdAck) // 0xA3,
	//JLENUMCASE(XMID_SetAmd) // 0xA2,
	//JLENUMCASE(XMID_SetAmdAck) // 0xA3,
	JLENUMCASE(XMID_ResetOrientation) // 0xA4,
	JLENUMCASE(XMID_ResetOrientationAck) // 0xA5,
	JLENUMCASE(XMID_ReqGnssStatus) // 0xA6,
	JLENUMCASE(XMID_GnssStatus) // 0xA7,
	JLENUMCASE(XMID_AdjustUtcTime) // 0xA8,
	JLENUMCASE(XMID_AdjustUtcTimeAck) // 0xA9,
	JLENUMCASE(XMID_ReqManufacturerId) // 0xAA,
	JLENUMCASE(XMID_ManufacturerId) // 0xAB,
	JLENUMCASE(XMID_ReqAccessControlList) // 0xAE,
	JLENUMCASE(XMID_AccessControlList) // 0xAF,
	//JLENUMCASE(XMID_SetAccessControlList) // 0xAE,
	//JLENUMCASE(XMID_SetAccessControlListAck) // 0xAF,
	JLENUMCASE(XMID_ScanChannels) // 0xB0,
	JLENUMCASE(XMID_ScanChannelsAck) // 0xB1,
	JLENUMCASE(XMID_EnableMaster) // 0xB2,
	JLENUMCASE(XMID_EnableMasterAck) // 0xB3,
	JLENUMCASE(XMID_DisableMaster) // 0xB4,
	JLENUMCASE(XMID_DisableMasterAck) // 0xB5,
	JLENUMCASE(XMID_ReqRadioChannel) // 0xB6,
	JLENUMCASE(XMID_ReqRadioChannelAck) // 0xB7,
	//JLENUMCASE(XMID_SetClientPriority) // 0xB8,
	//JLENUMCASE(XMID_SetClientPriorityAck) // 0xB9,
	JLENUMCASE(XMID_ReqClientPriority) // 0xB8,
	JLENUMCASE(XMID_ReqClientPriorityAck) // 0xB9,
	//JLENUMCASE(XMID_SetWirelessConfig) // 0xBA,
	//JLENUMCASE(XMID_SetWirelessConfigAck) // 0xBB,
	JLENUMCASE(XMID_ReqWirelessConfig) // 0xBA,
	JLENUMCASE(XMID_ReqWirelessConfigAck) // 0xBB,
	JLENUMCASE(XMID_UpdateBias) // 0xBC,
	JLENUMCASE(XMID_UpdateBiasAck) // 0xBD,
	JLENUMCASE(XMID_ToggleIoPins) // 0xBE,
	JLENUMCASE(XMID_ToggleIoPinsAck) // 0xBF,
	JLENUMCASE2(XMID_GotoOperational, "XMID_GotoOperational/XMID_ReqOutputConfiguration") // 0xC0,
	JLENUMCASE2(XMID_GotoOperationalAck, "XMID_GotoOperationalAck/XMID_ReqOutputConfigurationAck") // 0xC1,
	//JLENUMCASE(XMID_SetTransportMode) // 0xC2,
	//JLENUMCASE(XMID_SetTransportModeAck) // 0xC3,
	JLENUMCASE(XMID_ReqTransportMode) // 0xC2,
	JLENUMCASE(XMID_ReqTransportModeAck) // 0xC3,
	JLENUMCASE(XMID_AcceptMtw) // 0xC4,
	JLENUMCASE(XMID_AcceptMtwAck) // 0xC5,
	JLENUMCASE(XMID_RejectMtw) // 0xC6,
	JLENUMCASE(XMID_RejectMtwAck) // 0xC7,
	JLENUMCASE(XMID_InfoRequest) // 0xC8,
	JLENUMCASE(XMID_InfoRequestAck) // 0xC9,
	JLENUMCASE(XMID_ReqFrameRates) // 0xCA,
	JLENUMCASE(XMID_ReqFrameRatesAck) // 0xCB,
	JLENUMCASE(XMID_StartRecording) // 0xCC,
	JLENUMCASE(XMID_StartRecordingAck) // 0xCD,
	JLENUMCASE(XMID_StopRecording) // 0xCE,
	JLENUMCASE(XMID_StopRecordingAck) // 0xCF,
	//JLENUMCASE(XMID_ReqOutputConfiguration) // 0xC0,
	//JLENUMCASE(XMID_ReqOutputConfigurationAck) // 0xC1,
	//JLENUMCASE(XMID_SetOutputConfiguration) // 0xC0,
	//JLENUMCASE(XMID_SetOutputConfigurationAck) // 0xC1,
	JLENUMCASE(XMID_ReqOutputMode) // 0xD0,
	JLENUMCASE(XMID_ReqOutputModeAck) // 0xD1,
	//JLENUMCASE(XMID_SetOutputMode) // 0xD0,
	//JLENUMCASE(XMID_SetOutputModeAck) // 0xD1,
	JLENUMCASE(XMID_ReqOutputSettings) // 0xD2,
	JLENUMCASE(XMID_ReqOutputSettingsAck) // 0xD3,
	//JLENUMCASE(XMID_SetOutputSettings) // 0xD2,
	//JLENUMCASE(XMID_SetOutputSettingsAck) // 0xD3,
	JLENUMCASE(XMID_ReqOutputSkipFactor) // 0xD4,
	JLENUMCASE(XMID_ReqOutputSkipFactorAck) // 0xD5,
	//JLENUMCASE(XMID_SetOutputSkipFactor) // 0xD4,
	//JLENUMCASE(XMID_SetOutputSkipFactorAck) // 0xD5,
	//JLENUMCASE(XMID_ReqSyncInSettings) // 0xD6,
	//JLENUMCASE(XMID_ReqSyncInSettingsAck) // 0xD7,
	//JLENUMCASE(XMID_SetSyncInSettings) // 0xD6,
	//JLENUMCASE(XMID_SetSyncInSettingsAck) // 0xD7,
	//JLENUMCASE(XMID_ReqSyncOutSettings) // 0xD8,
	//JLENUMCASE(XMID_ReqSyncOutSettingsAck) // 0xD9,
	//JLENUMCASE(XMID_SetSyncOutSettings) // 0xD8,
	//JLENUMCASE(XMID_SetSyncOutSettingsAck) // 0xD9,
	JLENUMCASE(XMID_ReqErrorMode) // 0xDA,
	JLENUMCASE(XMID_ReqErrorModeAck) // 0xDB,
	//JLENUMCASE(XMID_SetErrorMode) // 0xDA,
	//JLENUMCASE(XMID_SetErrorModeAck) // 0xDB,
	JLENUMCASE(XMID_ReqTransmitDelay) // 0xDC,
	JLENUMCASE(XMID_ReqTransmitDelayAck) // 0xDD,
	//JLENUMCASE(XMID_SetTransmitDelay) // 0xDC,
	//JLENUMCASE(XMID_SetTransmitDelayAck) // 0xDD,
	JLENUMCASE(XMID_SetMfmResults) // 0xDE,
	JLENUMCASE(XMID_SetMfmResultsAck) // 0xDF,
	JLENUMCASE(XMID_ReqObjectAlignment) // 0xE0,
	JLENUMCASE(XMID_ReqObjectAlignmentAck) // 0xE1,
	//JLENUMCASE(XMID_SetObjectAlignment) // 0xE0,
	//JLENUMCASE(XMID_SetObjectAlignmentAck) // 0xE1,
	JLENUMCASE(XMID_ReqCanConfig)	// 0xE6
	JLENUMCASE(XMID_CanConfig)		// 0xE7
	//JLENUMCASE(XMID_SetCanConfig)	// 0xE6
	//JLENUMCASE(XMID_SetCanConfigAck)		// 0xE7
	JLENUMCASE(XMID_ReqCanOutputConfig) // 0xE8
	JLENUMCASE(XMID_CanOutputConfig) // 0xE9
	JLENUMCASE(XMID_ReqAlignmentRotation) // 0xEC,
	JLENUMCASE(XMID_ReqAlignmentRotationAck) // 0xED,
	//JLENUMCASE(XMID_SetAlignmentRotation) // 0xEC,
	//JLENUMCASE(XMID_SetAlignmentRotationAck) // 0xED,
	JLENUMCASE(XMID_ExtensionReserved1) // 0xEE,
	JLENUMCASE(XMID_ExtensionReserved2) // 0xEF,
	JLENUMCASE(XMID_SetDeviceIdContext) // 0xFE,
	JLENUMCASE(XMID_SetDeviceIdContextAck) // 0xFF

	JLENUMCASE(XMID_ReqHardwareVersion)
	JLENUMCASE(XMID_HardwareVersion)
	JLENUMCASE(XMID_Warning)
	JLENUMCASE(XMID_ReqDeviceCapabilities)
	JLENUMCASE(XMID_DeviceCapabilities)
	JLENUMCASE(XMID_DiscardRetransmissions)
	JLENUMCASE(XMID_DiscardRetransmissionsAck)
	JLENUMCASE(XMID_ReqErrorReport)
	JLENUMCASE(XMID_ErrorReport)
	JLENUMCASE(XMID_ReqGnssReceiverSettings)
	JLENUMCASE(XMID_ReqGnssReceiverSettingsAck)
	JLENUMCASE(XMID_ForwardGnssData)
	JLENUMCASE(XMID_ForwardGnssDataAck)
	JLENUMCASE(XMID_ReqProductVariant)
	JLENUMCASE(XMID_ProductVariant)
);

static void forceEnumExpanderInclusion()
{
	volatile bool blah = false;
	JLWRITEG("blah" << XRV_OK << blah);
	JLWRITEG("blah" << XMID_AcceptMtw << blah);
}
