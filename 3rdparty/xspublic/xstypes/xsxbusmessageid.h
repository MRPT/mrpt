
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

#ifndef XSXBUSMESSAGEID_H
#define XSXBUSMESSAGEID_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Xsens Xbus Message Identifiers
*/
enum XsXbusMessageId {
	XMID_InvalidMessage             = 0x00,

	// Config state messages
	XMID_ReqDid                     = 0x00,
	XMID_DeviceId                   = 0x01,
	XMID_Initbus                    = 0x02,
	XMID_InitBusResults             = 0x03,
	XMID_ReqPeriod                  = 0x04,
	XMID_ReqPeriodAck               = 0x05,
	XMID_SetPeriod                  = 0x04,
	XMID_SetPeriodAck               = 0x05,

	XMID_SetBid                     = 0x06,
	XMID_SetBidAck                  = 0x07,
	XMID_AutoStart                  = 0x06,
	XMID_AutoStartAck               = 0x07,
	XMID_BusPower                   = 0x08,
	XMID_BusPowerAck                = 0x09,

	XMID_ReqDataLength              = 0x0A,
	XMID_DataLength                 = 0x0B,
	XMID_ReqConfiguration           = 0x0C,
	XMID_Configuration              = 0x0D,
	XMID_RestoreFactoryDef          = 0x0E,
	XMID_RestoreFactoryDefAck       = 0x0F,

	XMID_GotoMeasurement            = 0x10,
	XMID_GotoMeasurementAck         = 0x11,
	XMID_ReqFirmwareRevision        = 0x12,
	XMID_FirmwareRevision           = 0x13,
	XMID_ReqUniqueId                = 0x14,
	XMID_UniqueId                   = 0x15,
	XMID_ReqBodypackMode            = 0x16,
	XMID_ReqBodypackAck             = 0x17,
	XMID_SetBodypackMode            = 0x16,
	XMID_SetBodypackModeAck         = 0x17,

	XMID_ReqBaudrate                = 0x18,
	XMID_ReqBaudrateAck             = 0x19,
	XMID_SetBaudrate                = 0x18,
	XMID_SetBaudrateAck             = 0x19,
	XMID_ReqProductVariant          = 0x1A,
	XMID_ProductVariant             = 0x1B,
	XMID_ReqProductCode             = 0x1C,
	XMID_ProductCode                = 0x1D,

	XMID_ReqHardwareVersion         = 0x1E,
	XMID_HardwareVersion            = 0x1F,

	XMID_ReqProcessingFlags         = 0x20,
	XMID_ReqProcessingFlagsAck      = 0x21,
	XMID_SetProcessingFlags         = 0x20,
	XMID_SetProcessingFlagsAck      = 0x21,

	XMID_SetNoRotation              = 0x22,
	XMID_SetNoRotationAck           = 0x23,

	XMID_RunSelfTest                = 0x24,
	XMID_SelfTestResults            = 0x25,

	XMID_ReqInputTrigger            = 0x26,
	XMID_ReqInputTriggerAck         = 0x27,
	XMID_SetInputTrigger            = 0x26,
	XMID_SetInputTriggerAck         = 0x27,

	XMID_ReqOutputTrigger           = 0x28,
	XMID_ReqOutputTriggerAck        = 0x29,
	XMID_SetOutputTrigger           = 0x28,
	XMID_SetOutputTriggerAck        = 0x29,

	XMID_SetSyncStationMode         = 0x2A,		//!< Request the current sync station mode
	XMID_SetSyncStationModeAck      = 0x2B,		//!< Message contains the current sync station mode
	XMID_ReqSyncStationMode         = 0x2A,		//!< Set the current sync station mode
	XMID_ReqSyncStationModeAck      = 0x2B,		//!< Acknowledge of setting the current sync station mode

	XMID_SetSyncConfiguration       = 0x2C,
	XMID_SetSyncConfigurationAck    = 0x2D,
	XMID_ReqSyncConfiguration       = 0x2C,
	XMID_SyncConfiguration          = 0x2D,

	XMID_DriverDisconnect           = 0x2E,
	XMID_DriverDisconnectAck        = 0x2F,

	XMID_GotoConfig                 = 0x30,
	XMID_GotoConfigAck              = 0x31,
	XMID_MtData                     = 0x32,
	XMID_BusData                    = 0x32,

	// Manual
	XMID_PrepareData                = 0x32,
	XMID_ReqData                    = 0x34,
	XMID_ReqDataAck                 = 0x35,

	XMID_MtData2                    = 0x36,
	XMID_MtData2Ack	                = 0x37,

	XMID_RequestControl             = 0x38,
	XMID_RequestControlAck          = 0x39,

	XMID_SetDataPort                = 0x3A,
	XMID_SetDataPortAck             = 0x3B,

	XMID_ReqRetransmission          = 0x3C,
	XMID_ReqRetransmissionAck       = 0x3D,

	// Wakeup state messages
	XMID_Wakeup                     = 0x3E,
	XMID_WakeupAck                  = 0x3F,

	// Valid in all states
	XMID_Reset                      = 0x40,
	XMID_ResetAck                   = 0x41,
	XMID_Error                      = 0x42,
	XMID_Warning                    = 0x43,
	// end Valid in all states

	XMID_XmPowerOff                 = 0x44,

	// Wireless
	XMID_MasterIndication           = 0x46,

	XMID_ReqOptionFlags             = 0x48,
	XMID_ReqOptionFlagsAck          = 0x49,
	XMID_SetOptionFlags             = 0x48,
	XMID_SetOptionFlagsAck          = 0x49,
	XMID_ReqStealthMode             = 0x4A,
	XMID_StealthMode                = 0x4B,
	XMID_SetStealthMode             = 0x4A,
	XMID_SetStealthModeAck          = 0x4B,

	XMID_UserInterface              = 0x4C,
	XMID_UserInterfaceAck           = 0x4D,

	XMID_EndOfRecording             = 0x4E,
	XMID_EndOfRecordingAck          = 0x4F,

	XMID_GotoTransparentMode        = 0x50,
	XMID_GotoTransparentModeAck     = 0x51,

	XMID_ReqDeviceCapabilities      = 0x52,
	XMID_DeviceCapabilities         = 0x53,

	XMID_DiscardRetransmissions     = 0x54,
	XMID_DiscardRetransmissionsAck  = 0x55,

	XMID_RunFactoryTest             = 0x56,
	XMID_FactoryTestResults         = 0x57,
	XMID_FactoryTestConnect         = 0x58,
	XMID_FactoryTestConnectAck      = 0x59,

	XMID_SetDataOutputDelay         = 0x5A,
	XMID_SetDataOutputDelayAck      = 0x5B,

	XMID_SetBodypackConfigFile      = 0x5C,
	XMID_SetBodypackConfigFileAck   = 0x5D,

	XMID_ReqObrStatus               = 0x5E,
	XMID_ObrStatus                  = 0x5F,

	XMID_SetUtcTime                 = 0x60,
	XMID_ReqUtcTime                 = 0x60,
	XMID_SetUtcTimeAck              = 0x61,
	XMID_UtcTime                    = 0x61,

	XMID_FactoryTestSensorTiming        = 0x60,
	XMID_FactoryTestSensorTimingResults = 0x61,

	XMID_ReqAvailableFilterProfiles = 0x62,		//!< Request the available filter profiles
	XMID_AvailableFilterProfiles    = 0x63,		//!< Message contains the available filter profiles

	XMID_ReqFilterProfile           = 0x64,		//!< Request the current filter profile
	XMID_ReqFilterProfileAck        = 0x65,		//!< Message contains the current filter profile
	XMID_SetFilterProfile           = 0x64,		//!< Set the current filter profile
	XMID_SetFilterProfileAck        = 0x65,		//!< Acknowledge of setting the current filter profile

	XMID_ReqGravityMagnitude        = 0x66,
	XMID_ReqGravityMagnitudeAck     = 0x67,
	XMID_SetGravityMagnitude        = 0x66,
	XMID_SetGravityMagnitudeAck     = 0x67,

	XMID_ReqGnssLeverArm            = 0x68,
	XMID_ReqGnssLeverArmAck         = 0x69,
	XMID_SetGnssLeverArm            = 0x68,
	XMID_SetGnssLeverArmAck         = 0x69,

	XMID_ReqMagneticField           = 0x6A,
	XMID_ReqMagneticFieldAck        = 0x6B,
	XMID_SetMagneticField           = 0x6A,
	XMID_SetMagneticFieldAck        = 0x6B,

	XMID_ReqReplayMode              = 0x6C,
	XMID_ReqReplayModeAck           = 0x6D,
	XMID_SetReplayMode              = 0x6C,
	XMID_SetReplayModeAck           = 0x6D,

	XMID_ReqLatLonAlt               = 0x6E,
	XMID_ReqLatLonAltAck            = 0x6F,
	XMID_SetLatLonAlt               = 0x6E,
	XMID_SetLatLonAltAck            = 0x6F,

	XMID_KeepAlive                  = 0x70,
	XMID_KeepAliveAck               = 0x71,

	XMID_CloseConnection            = 0x72,
	XMID_CloseConnectionAck         = 0x73,

	XMID_IccCommand                 = 0x74,
	XMID_IccCommandAck              = 0x75,

	XMID_ReqGnssPlatform            = 0x76,
	XMID_ReqGnssPlatformAck         = 0x77,
	XMID_SetGnssPlatform            = 0x76,
	XMID_SetGnssPlatformAck         = 0x77,

	XMID_ReqConnectionSettings      = 0x78,
	XMID_ReqConnectionSettingsAck   = 0x79,
	XMID_SetConnectionSettings      = 0x78,
	XMID_SetConnectionSettingsAck   = 0x79,

	XMID_BodyPackBundle             = 0x7A,
	XMID_BodyPackBundleAck          = 0x7B,

	XMID_ReqStationOptions          = 0x7C,
	XMID_ReqStationOptionsAck       = 0x7D,

	XMID_ReqErrorReport             = 0x7E,
	XMID_ErrorReport                = 0x7F,

	XMID_ReqXmErrorMode             = 0x82,
	XMID_ReqXmErrorModeAck          = 0x83,
	XMID_SetXmErrorMode             = 0x82,
	XMID_SetXmErrorModeAck          = 0x83,

	XMID_ReqBufferSize              = 0x84,
	XMID_ReqBufferSizeAck           = 0x85,
	XMID_SetBufferSize              = 0x84,
	XMID_SetBufferSizeAck           = 0x85,

	XMID_ReqHeading                 = 0x82,
	XMID_ReqHeadingAck              = 0x83,
	XMID_SetHeading                 = 0x82,
	XMID_SetHeadingAck              = 0x83,

	XMID_ReqLocationId              = 0x84,
	XMID_ReqLocationIdAck           = 0x85,
	XMID_SetLocationId              = 0x84,
	XMID_SetLocationIdAck           = 0x85,

	XMID_ReqExtOutputMode           = 0x86,
	XMID_ReqExtOutputModeAck        = 0x87,
	XMID_SetExtOutputMode           = 0x86,
	XMID_SetExtOutputModeAck        = 0x87,

	XMID_ReqBatteryLevel            = 0x88,
	XMID_Batterylevel               = 0x89,

	XMID_ReqInitTrackMode           = 0x88,
	XMID_ReqInitTrackModeAck        = 0x89,
	XMID_SetInitTrackMode           = 0x88,
	XMID_SetInitTrackModeAck        = 0x89,

	XMID_ReqMasterSettings          = 0x8A,
	XMID_MasterSettings             = 0x8B,

	XMID_StoreFilterState           = 0x8A,
	XMID_StoreFilterStateAck        = 0x8B,

	XMID_ReqPortConfig				= 0x8C,
	XMID_SetPortConfig				= 0x8C,
	XMID_PortConfig					= 0x8D,
	XMID_SetPortConfigAck			= 0x8D,

	XMID_ReqStringOutputType        = 0x8E,
	XMID_ReqStringOutputTypeAck     = 0x8F,
	XMID_SetStringOutputType        = 0x8E,
	XMID_SetStringOutputTypeAck     = 0x8F,

	XMID_ReqStringOutputConfig		= 0x8E,
	XMID_ReqStringOutputConfigAck	= 0x8F,
	XMID_SetStringOutputConfig		= 0x8E,
	XMID_SetStringOutputConfigAck	= 0x8F,

	XMID_ReqEmts                    = 0x90,
	XMID_EmtsData                   = 0x91,
	XMID_UpdateFilterProfile        = 0x92,
	XMID_UpdateFilterProfileAck     = 0x93,

	XMID_RestoreEmts                = 0x94,
	XMID_RestoreEmtsAck             = 0x95,
	XMID_StoreEmts                  = 0x96,
	XMID_StoreEmtsAck               = 0x97,

	XMID_ClockSyncCommand			= 0x9A,
	XMID_ClockSyncCommandAck		= 0x9B,

	XMID_ReqActiveClockCorrection      = 0x9C,
	XMID_ActiveClockCorrection         = 0x9D,
	XMID_StoreActiveClockCorrection    = 0x9E,
	XMID_StoreActiveClockCorrectionAck = 0x9F,

	XMID_ReqFilterSettings          = 0xA0,
	XMID_ReqFilterSettingsAck       = 0xA1,
	XMID_SetFilterSettings          = 0xA0,
	XMID_SetFilterSettingsAck       = 0xA1,
	XMID_ReqAmd                     = 0xA2,
	XMID_ReqAmdAck                  = 0xA3,
	XMID_SetAmd                     = 0xA2,
	XMID_SetAmdAck                  = 0xA3,
	XMID_ResetOrientation           = 0xA4,
	XMID_ResetOrientationAck        = 0xA5,

	XMID_ReqGnssStatus              = 0xA6,
	XMID_GnssStatus                 = 0xA7,

	XMID_AdjustUtcTime              = 0xA8,
	XMID_AdjustUtcTimeAck           = 0xA9,

	XMID_ReqManufacturerId          = 0xAA,
	XMID_SetManufacturerId          = 0xAA,
	XMID_ManufacturerId             = 0xAB,
	XMID_SetManufacturerIdAck       = 0xAB,

	XMID_ReqGnssReceiverSettings	= 0xAC,
	XMID_ReqGnssReceiverSettingsAck	= 0xAD,
	XMID_SetGnssReceiverSettings	= 0xAC,
	XMID_SetGnssReceiverSettingsAck	= 0xAD,

	XMID_ReqAccessControlList       = 0xAE,
	XMID_AccessControlList          = 0xAF,
	XMID_SetAccessControlList       = 0xAE,
	XMID_SetAccessControlListAck    = 0xAF,

	// Wireless
	XMID_ScanChannels               = 0xB0,
	XMID_ScanChannelsAck            = 0xB1,
	XMID_EnableMaster               = 0xB2,
	XMID_EnableMasterAck            = 0xB3,
	XMID_DisableMaster              = 0xB4,
	XMID_DisableMasterAck           = 0xB5,
	XMID_ReqRadioChannel            = 0xB6,
	XMID_ReqRadioChannelAck         = 0xB7,
	XMID_SetClientPriority          = 0xB8,
	XMID_SetClientPriorityAck       = 0xB9,
	XMID_ReqClientPriority          = 0xB8,
	XMID_ReqClientPriorityAck       = 0xB9,
	XMID_SetWirelessConfig          = 0xBA,
	XMID_SetWirelessConfigAck       = 0xBB,
	XMID_ReqWirelessConfig          = 0xBA,
	XMID_ReqWirelessConfigAck       = 0xBB,
	XMID_UpdateBias                 = 0xBC,
	XMID_UpdateBiasAck              = 0xBD,
	XMID_ToggleIoPins               = 0xBE,
	XMID_ToggleIoPinsAck            = 0xBF,

	XMID_GotoOperational            = 0xC0,
	XMID_GotoOperationalAck         = 0xC1,

	XMID_SetTransportMode           = 0xC2,
	XMID_SetTransportModeAck        = 0xC3,
	XMID_ReqTransportMode           = 0xC2,
	XMID_ReqTransportModeAck        = 0xC3,

	XMID_AcceptMtw                  = 0xC4,
	XMID_AcceptMtwAck               = 0xC5,
	XMID_RejectMtw                  = 0xC6,
	XMID_RejectMtwAck               = 0xC7,
	XMID_InfoRequest                = 0xC8,
	XMID_InfoRequestAck             = 0xC9,

	XMID_ReqFrameRates              = 0xCA,
	XMID_ReqFrameRatesAck           = 0xCB,

	XMID_StartRecording             = 0xCC,
	XMID_StartRecordingAck          = 0xCD,
	XMID_StopRecording              = 0xCE,
	XMID_StopRecordingAck           = 0xCF,
	// End Wireless

	XMID_ReqOutputConfiguration     = 0xC0,
	XMID_ReqOutputConfigurationAck  = 0xC1,
	XMID_SetOutputConfiguration     = 0xC0,
	XMID_SetOutputConfigurationAck	= 0xC1,

	XMID_ReqOutputMode              = 0xD0,
	XMID_ReqOutputModeAck           = 0xD1,
	XMID_SetOutputMode              = 0xD0,
	XMID_SetOutputModeAck           = 0xD1,

	XMID_ReqOutputSettings          = 0xD2,
	XMID_ReqOutputSettingsAck       = 0xD3,
	XMID_SetOutputSettings          = 0xD2,
	XMID_SetOutputSettingsAck       = 0xD3,

	XMID_ReqOutputSkipFactor        = 0xD4,
	XMID_ReqOutputSkipFactorAck     = 0xD5,
	XMID_SetOutputSkipFactor        = 0xD4,
	XMID_SetOutputSkipFactorAck     = 0xD5,

	XMID_ReqErrorMode               = 0xDA,
	XMID_ReqErrorModeAck            = 0xDB,
	XMID_SetErrorMode               = 0xDA,
	XMID_SetErrorModeAck            = 0xDB,

	XMID_ReqTransmitDelay           = 0xDC,
	XMID_ReqTransmitDelayAck        = 0xDD,
	XMID_SetTransmitDelay           = 0xDC,
	XMID_SetTransmitDelayAck        = 0xDD,

	XMID_SetMfmResults              = 0xDE,
	XMID_SetMfmResultsAck           = 0xDF,

	XMID_ReqObjectAlignment         = 0xE0,
	XMID_ReqObjectAlignmentAck      = 0xE1,
	XMID_SetObjectAlignment         = 0xE0,
	XMID_SetObjectAlignmentAck      = 0xE1,

	XMID_ForwardGnssData			= 0xE2,
	XMID_ForwardGnssDataAck			= 0xE3,

	XMID_ReqCanConfig				= 0xE6,
	XMID_SetCanConfig				= 0xE6,
	XMID_CanConfig					= 0xE7,
	XMID_SetCanConfigAck			= 0xE7,
	XMID_ReqCanOutputConfig			= 0xE8,
	XMID_SetCanOutputConfig			= 0xE8,
	XMID_CanOutputConfig			= 0xE9,
	XMID_SetCanOutputConfigAck		= 0xE9,

	XMID_ReqAlignmentRotation       = 0xEC,
	XMID_ReqAlignmentRotationAck    = 0xED,
	XMID_SetAlignmentRotation       = 0xEC,
	XMID_SetAlignmentRotationAck    = 0xED,

	XMID_ExtensionReserved1         = 0xEE,
	XMID_ExtensionReserved2         = 0xEF,

	XMID_SetDeviceIdContext         = 0xFE,
	XMID_SetDeviceIdContextAck      = 0xFF
};
/*! @} */
typedef enum XsXbusMessageId XsXbusMessageId;

#endif
