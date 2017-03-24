/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSXBUSMESSAGEID_H
#define XSXBUSMESSAGEID_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Xsens Xbus Message Identifiers
*/
enum XsXbusMessageId {
	XMID_InvalidMessage           	=0x00,

	// Wakeup state messages
	XMID_Wakeup                   	=0x3E,
	XMID_WakeupAck                	=0x3F,

	// Config state messages
	XMID_ReqDid                   	=0x00,
	XMID_DeviceId                 	=0x01,
	XMID_Initbus                  	=0x02,
	XMID_InitBusResults           	=0x03,
	XMID_ReqPeriod                	=0x04,
	XMID_ReqPeriodAck             	=0x05,
	XMID_SetPeriod                	=0x04,
	XMID_SetPeriodAck             	=0x05,
	// XbusMaster
	XMID_SetBid                   	=0x06,
	XMID_SetBidAck                	=0x07,
	XMID_AutoStart                	=0x06,
	XMID_AutoStartAck             	=0x07,
	XMID_BusPower                 	=0x08,
	XMID_BusPowerAck              	=0x09,
	// End XbusMaster=
	XMID_ReqDataLength            	=0x0A,
	XMID_DataLength               	=0x0B,
	XMID_ReqConfiguration         	=0x0C,
	XMID_Configuration            	=0x0D,
	XMID_RestoreFactoryDef        	=0x0E,
	XMID_RestoreFactoryDefAck     	=0x0F,

	XMID_GotoMeasurement          	=0x10,
	XMID_GotoMeasurementAck       	=0x11,
	XMID_ReqFirmwareRevision      	=0x12,
	XMID_FirmwareRevision         	=0x13,
	// XbusMaster
	XMID_ReqBluetoothDisable      	=0x14,
	XMID_ReqBluetoothDisableAck   	=0x15,
	XMID_DisableBluetooth         	=0x14,
	XMID_DisableBluetoothAck      	=0x15,
	XMID_ReqXmOutputMode          	=0x16,
	XMID_ReqXmOutputModeAck       	=0x17,
	XMID_SetXmOutputMode          	=0x16,
	XMID_SetXmOutputModeAck       	=0x17,
	// End XbusMaster
	XMID_ReqBaudrate              	=0x18,
	XMID_ReqBaudrateAck           	=0x19,
	XMID_SetBaudrate              	=0x18,
	XMID_SetBaudrateAck           	=0x19,
	// XbusMaster
	XMID_ReqSyncMode              	=0x1A,
	XMID_ReqSyncModeAck           	=0x1B,
	XMID_SetSyncMode              	=0x1A,
	XMID_SetSyncModeAck           	=0x1B,
	// End XbusMaster
	XMID_ReqProductCode           	=0x1C,
	XMID_ProductCode              	=0x1D,

	XMID_ReqProcessingFlags       	=0x20,
	XMID_ReqProcessingFlagsAck    	=0x21,
	XMID_SetProcessingFlags       	=0x20,
	XMID_SetProcessingFlagsAck    	=0x21,

	XMID_ReqInputTrigger          	=0x26,
	XMID_ReqInputTriggerAck       	=0x27,
	XMID_SetInputTrigger          	=0x26,
	XMID_SetInputTriggerAck       	=0x27,

	XMID_ReqOutputTrigger         	=0x28,
	XMID_ReqOutputTriggerAck      	=0x29,
	XMID_SetOutputTrigger         	=0x28,
	XMID_SetOutputTriggerAck      	=0x29,

	XMID_SetSyncBoxMode           	=0x2A,
	XMID_SetSyncBoxModeAck        	=0x2B,
	XMID_ReqSyncBoxMode           	=0x2A,
	XMID_ReqSyncBoxModeAck        	=0x2B,

	XMID_SetSyncConfiguration       =0x2C,
	XMID_SetSyncConfigurationAck    =0x2D,
	XMID_ReqSyncConfiguration       =0x2C,
	XMID_SyncConfiguration          =0x2D,

	XMID_DriverDisconnect         	=0x2E,
	XMID_DriverDisconnectAck      	=0x2F,

	// XbusMaster
	XMID_XmPowerOff               	=0x44,
	// End XbusMaster

	XMID_ReqOutputConfiguration   	=0xC0,
	XMID_ReqOutputConfigurationAck	=0xC1,
	XMID_SetOutputConfiguration   	=0xC0,
	XMID_SetOutputConfigurationAck	=0xC1,

	XMID_ReqOutputMode            	=0xD0,
	XMID_ReqOutputModeAck         	=0xD1,
	XMID_SetOutputMode            	=0xD0,
	XMID_SetOutputModeAck         	=0xD1,

	XMID_ReqOutputSettings        	=0xD2,
	XMID_ReqOutputSettingsAck     	=0xD3,
	XMID_SetOutputSettings        	=0xD2,
	XMID_SetOutputSettingsAck     	=0xD3,

	XMID_ReqOutputSkipFactor      	=0xD4,
	XMID_ReqOutputSkipFactorAck   	=0xD5,
	XMID_SetOutputSkipFactor      	=0xD4,
	XMID_SetOutputSkipFactorAck   	=0xD5,

	XMID_ReqSyncInSettings        	=0xD6,
	XMID_ReqSyncInSettingsAck     	=0xD7,
	XMID_SetSyncInSettings        	=0xD6,
	XMID_SetSyncInSettingsAck     	=0xD7,

	XMID_ReqSyncOutSettings       	=0xD8,
	XMID_ReqSyncOutSettingsAck    	=0xD9,
	XMID_SetSyncOutSettings       	=0xD8,
	XMID_SetSyncOutSettingsAck    	=0xD9,

	XMID_ReqErrorMode             	=0xDA,
	XMID_ReqErrorModeAck          	=0xDB,
	XMID_SetErrorMode             	=0xDA,
	XMID_SetErrorModeAck          	=0xDB,

	XMID_ReqTransmitDelay         	=0xDC,
	XMID_ReqTransmitDelayAck      	=0xDD,
	XMID_SetTransmitDelay         	=0xDC,
	XMID_SetTransmitDelayAck      	=0xDD,

	XMID_SetMfmResults              =0xDE,
	XMID_SetMfmResultsAck           =0xDF,

	XMID_ReqObjectAlignment       	=0xE0,
	XMID_ReqObjectAlignmentAck    	=0xE1,
	XMID_SetObjectAlignment       	=0xE0,
	XMID_SetObjectAlignmentAck    	=0xE1,

	XMID_ReqAlignmentRotation       =0xEC,
	XMID_ReqAlignmentRotationAck    =0xED,
    XMID_SetAlignmentRotation       =0xEC,
    XMID_SetAlignmentRotationAck    =0xED,

	// Xbus Master
	XMID_ReqXmErrorMode           	=0x82,
	XMID_ReqXmErrorModeAck        	=0x83,
	XMID_SetXmErrorMode           	=0x82,
	XMID_SetXmErrorModeAck        	=0x83,

	XMID_ReqBufferSize            	=0x84,
	XMID_ReqBufferSizeAck         	=0x85,
	XMID_SetBufferSize            	=0x84,
	XMID_SetBufferSizeAck         	=0x85,
	// End Xbus Master

	XMID_ReqHeading               	=0x82,
	XMID_ReqHeadingAck            	=0x83,
	XMID_SetHeading               	=0x82,
	XMID_SetHeadingAck            	=0x83,

	XMID_ReqMagneticField         	=0x6A,
	XMID_ReqMagneticFieldAck      	=0x6B,
	XMID_SetMagneticField         	=0x6A,
	XMID_SetMagneticFieldAck      	=0x6B,

	XMID_ReqLocationid            	=0x84,
	XMID_ReqLocationidAck         	=0x85,
	XMID_SetLocationId            	=0x84,
	XMID_SetLocationIdAck         	=0x85,

	XMID_ReqExtOutputMode         	=0x86,
	XMID_ReqExtOutputModeAck      	=0x87,
	XMID_SetExtOutputMode         	=0x86,
	XMID_SetExtOutputModeAck      	=0x87,
	
	XMID_ReqStringOutputType		=0x8E,
	XMID_ReqStringOutputTypeAck		=0x8F,
	XMID_SetStringOutputType		=0x8E,
	XMID_SetStringOutputTypeAck		=0x8F,

	// XbusMaster
	XMID_ReqBatteryLevel          	=0x88,
	XMID_Batterylevel             	=0x89,
	// End XbusMaster

	XMID_ReqInitTrackMode         	=0x88,
	XMID_ReqInitTrackModeAck      	=0x89,
	XMID_SetInitTrackMode         	=0x88,
	XMID_SetInitTrackModeAck      	=0x89,

	XMID_ReqMasterSettings        	=0x8A,
	XMID_MasterSettings           	=0x8B,

	XMID_StoreFilterState          	=0x8A,
	XMID_StoreFilterStateAck       	=0x8B,

	XMID_SetUtcTime               	=0x60,
	XMID_ReqUtcTime               	=0x60,
	XMID_SetUtcTimeAck             	=0x61,
	XMID_UtcTime                  	=0x61,
	XMID_AdjustUtcTime				=0xA8,
	XMID_AdjustUtcTimeAck			=0xA9,

	XMID_ReqActiveClockCorrection  	=0x9C,
	XMID_ActiveClockCorrection    	=0x9D,
	XMID_StoreActiveClockCorrection =0x9E,
	XMID_StoreActiveClockCorrectionAck = 0x9F,

	XMID_ReqAvailableFilterProfiles	=0x62,
	XMID_AvailableFilterProfiles	=0x63,

	XMID_ReqFilterProfile          	=0x64,
	XMID_ReqFilterProfileAck        =0x65,
	XMID_SetFilterProfile           =0x64,
	XMID_SetFilterProfileAck        =0x65,

	// the name 'scenario' has been deprecated for this use
	XMID_ReqAvailableScenarios    	=XMID_ReqAvailableFilterProfiles,
	XMID_AvailableScenarios       	=XMID_AvailableFilterProfiles,
	XMID_ReqScenario              	=XMID_ReqFilterProfile,
	XMID_ReqScenarioAck           	=XMID_ReqFilterProfileAck,
	XMID_SetScenario              	=XMID_SetFilterProfile,
	XMID_SetScenarioAck           	=XMID_SetFilterProfileAck,

	XMID_ReqGravityMagnitude      	=0x66,
	XMID_ReqGravityMagnitudeAck   	=0x67,
	XMID_SetGravityMagnitude      	=0x66,
	XMID_SetGravityMagnitudeAck   	=0x67,

	XMID_ReqGpsLeverArm           	=0x68,
	XMID_ReqGpsLeverArmAck        	=0x69,
	XMID_SetGpsLeverArm           	=0x68,
	XMID_SetGpsLeverArmAck        	=0x69,

	XMID_ReqLatLonAlt             	=0x6E,
	XMID_ReqLatLonAltAck          	=0x6F,
	XMID_SetLatLonAlt             	=0x6E,
	XMID_SetLatLonAltAck          	=0x6F,

	// Measurement state
	XMID_GotoConfig               	=0x30,
	XMID_GotoConfigAck            	=0x31,
	XMID_BusData                  	=0x32,
	XMID_MtData                   	=0x32,

	XMID_SetNoRotation            	=0x22,
	XMID_SetNoRotationAck         	=0x23,

	XMID_RunSelfTest              	=0x24,
	XMID_SelfTestResults          	=0x25,

	// Manual
	XMID_PrepareData              	=0x32,
	XMID_ReqData                  	=0x34,
	XMID_ReqDataAck               	=0x35,

	XMID_MtData2                  	=0x36,

	// Valid in all states
	XMID_Reset                    	=0x40,
	XMID_ResetAck                 	=0x41,
	XMID_Error                    	=0x42,
	// Wireless
	XMID_MasterIndication         	=0x46,

	XMID_ReqFilterSettings        	=0xA0,
	XMID_ReqFilterSettingsAck     	=0xA1,
	XMID_SetFilterSettings        	=0xA0,
	XMID_SetFilterSettingsAck     	=0xA1,
	XMID_ReqAmd                   	=0xA2,
	XMID_ReqAmdAck                	=0xA3,
	XMID_SetAmd                   	=0xA2,
	XMID_SetAmdAck                	=0xA3,
	XMID_ResetOrientation         	=0xA4,
	XMID_ResetOrientationAck      	=0xA5,

	XMID_ReqGpsStatus             	=0xA6,
	XMID_GpsStatus                	=0xA7,

	XMID_WriteDeviceId			  	=0xB0,
	XMID_WriteDeviceIdAck         	=0xB1,
	XMID_WriteSecurityKey         	=0xB2,
	XMID_WriteSecurityKeyAck      	=0xB3,
	XMID_ProtectFlash   		  	=0xB4,
	XMID_ProtectFlashAck 		  	=0xB5,
	XMID_ReqSecurityCheck		  	=0xB6,
	XMID_SecurityCheck			  	=0xB7,

	// Wireless
	XMID_ScanChannels             	=0xB0,
	XMID_ScanChannelsAck          	=0xB1,
	XMID_EnableMaster             	=0xB2,
	XMID_EnableMasterAck          	=0xB3,
	XMID_DisableMaster            	=0xB4,
	XMID_DisableMasterAck         	=0xB5,
	XMID_ReqRadioChannel			=0xB6,
	XMID_ReqRadioChannelAck			=0xB7,
	XMID_SetClientPriority        	=0xB8,
	XMID_SetClientPriorityAck     	=0xB9,
	XMID_ReqClientPriority        	=0xB8,
	XMID_ReqClientPriorityAck     	=0xB9,
	XMID_SetWirelessConfig        	=0xBA,
	XMID_SetWirelessConfigAck     	=0xBB,
	XMID_ReqWirelessConfig        	=0xBA,
	XMID_ReqWirelessConfigAck     	=0xBB,
	XMID_UpdateBias               	=0xBC,
	XMID_UpdateBiasAck            	=0xBD,
	XMID_ToggleIoPins				=0xBE,
	XMID_ToggleIoPinsAck			=0xBF,

	XMID_SetTransportMode         	=0xC2,
	XMID_SetTransportModeAck      	=0xC3,
	XMID_ReqTransportMode         	=0xC2,
	XMID_ReqTransportModeAck      	=0xC3,

	XMID_AcceptMtw                	=0xC4,
	XMID_AcceptMtwAck             	=0xC5,
	XMID_RejectMtw                	=0xC6,
	XMID_RejectMtwAck             	=0xC7,
	XMID_InfoRequest              	=0xC8,
	XMID_InfoRequestAck           	=0xC9,

	XMID_ReqFrameRates            	=0xCA,
	XMID_ReqFrameRatesAck         	=0xCB,

	XMID_StartRecording           	=0xCC,
	XMID_StartRecordingAck        	=0xCD,
	XMID_StopRecording            	=0xCE,
	XMID_StopRecordingAck         	=0xCF,

	XMID_InfoBatteryLevel         	=0x49,
	XMID_InfoTemperature          	=0x4A,

	XMID_GotoOperational          	=0xC0,
	XMID_GotoOperationalAck       	=0xC1,
	// End Wireless

	XMID_ReqEmts                  	=0x90,
	XMID_EmtsData                 	=0x91,

	XMID_RestoreEmts			  	=0x94,
	XMID_RestoreEmtsAck			  	=0x95,
	XMID_StoreEmts			      	=0x96,
	XMID_StoreEmtsAck			  	=0x97,

	XMID_GotoTransparentMode	  	=0x50,
	XMID_GotoTransparentModeAck	  	=0x51
};
/*! @} */
typedef enum XsXbusMessageId XsXbusMessageId;

#endif // file guard
