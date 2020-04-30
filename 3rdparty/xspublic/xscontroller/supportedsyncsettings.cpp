
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

#include "supportedsyncsettings.h"

#include "synclinemk4.h"
#include "synclinegmt.h"
#include <set>
#include <xstypes/xssyncsettingarray.h>

namespace Synchronization
{

/*! \brief Returns the list of the supported synchronizations settings for the specified device id
	Each item in the list represent one possible function-line settings.
*/
XsSyncSettingArray supportedSyncSettings(XsDeviceId deviceId)
{
	if (deviceId.isAwindaXDongle() ||
		deviceId.isBodyPack())
	{
		return XsSyncSettingArray();
	}

	if (deviceId.isAwindaX())
	{
		return supportedSyncSettingsForAwindaBaseStation();
	}
	else if (deviceId.isSyncStationX())
	{
		return supportedSyncSettingsForAwindaBaseStation();
	}
	else if (deviceId.isMtMark5() && deviceId.isMtigX10())
	{
		return supportedSyncSettingsForMark5MtigX10Device();
	}
	else if (deviceId.isMtig())
	{
		return supportedSyncSettingsForMtigDevice();
	}
	else if (deviceId.isMtiX() && deviceId.isGnss())
	{
		return supportedSyncSettingsForMti7Device();
	}
	else if (deviceId.isMti6X0() && deviceId.isGnss())
	{
		return supportedSyncSettingsForMti670Device();
	}
	else if (deviceId.isMti6X0())
	{
		return supportedSyncSettingsForMt6x0Device();
	}
	else if (deviceId.isMtiX())
	{
		return supportedSyncSettingsForMtiXDevice();
	}
	else if (deviceId.isMti() || deviceId.isMtig())
	{
		return supportedSyncSettingsForMtiDevice();
	}
	else if (deviceId.isGnss())
	{
		return supportedSyncSettingsForGnssDevice();
	}
	else if (deviceId.isMtx2())
	{
		return supportedSyncSettingsForMtx2Device();
	}
	else if (deviceId.isMtw2())
	{
		return supportedSyncSettingsForMtx2Device();
	}
	else
	{
		return XsSyncSettingArray();
	}
}

/*! \returns true if the specified device id supports sync settings, false otherwise
*/
bool supportsSyncSettings(XsDeviceId deviceId)
{
	if (supportedSyncSettings(deviceId).size())
		return true;
	else
		return false;
}

/*! \brief Return true if \a setting1 is compatible with \a setting2 for a device with \a deviceId
*/
bool isCompatibleSyncSetting(XsDeviceId deviceId, XsSyncSetting const & setting1, XsSyncSetting const & setting2) {
	if (deviceId.isAwindaX())
	{
		return isAwindaSettingCompatible(setting1, setting2);
	}
	else if (deviceId.isSyncStationX())
	{
		return isSyncStationSettingCompatible(setting1, setting2);
	}
	else
	{
		return true; // Always compatible for devices other than awinda for now
	}
}

/*! \returns the time resolution in microseconds for a device with device id \a deviceId
	For example if the precision is 1 millisecond, 1000 is returned
	\note Currently MVN studio only supports 1 or 1000
*/
unsigned int timeResolutionInMicroseconds(XsDeviceId deviceId)
{
	if (deviceId.isAwindaX())
		return awindaTimeResolutionInMicroseconds();
	else if (deviceId.isSyncStationX())
		return syncStationTimeResolutionInMicroseconds();
	else if (deviceId.isMti() || deviceId.isMtig())
		return mtiTimeResolutionInMicroseconds();
	else if (deviceId.isMtx2())
		return mtx2TimeResolutionInMicroseconds();
	else
		return 1;
}

/*! \brief get list of supported synchronizations settings for an Mtx2Device */
XsSyncSettingArray supportedSyncSettingsForMtx2Device()
{
	XsSyncSettingArray settings;
	return settings;
}

/*! \brief get list of supported synchronizations settings for an MtiBaseDevice */
#define SUPPORT_XSL4_BiIn 0 // Bidirectional 1 <- In is not supported anymore
XsSyncSettingArray supportedSyncSettingsForMtiDevice()
{
	XsSyncSetting s;
	XsSyncSettingArray settings;

	//enable/disabled parameters per functions
	s.m_polarity = XSP_RisingEdge;
	s.m_offset = 1;
	s.m_skipFactor = 1;
	s.m_triggerOnce = 0;
	s.m_skipFirst = 0;
	s.m_pulseWidth = 0;
	s.m_clockPeriod = 0;

	s.m_function	= XSF_TriggerIndication;
	s.m_line		= xsl4ToXsl(XSL4_In);
	settings.push_back(s);

#if SUPPORT_XSL4_BiIn == 1
	s.m_line		= xsl4ToXsl(XSL4_BiIn);
	settings.push_back(s);
#endif

	s.m_function	= XSF_SendLatest;
	s.m_line		= xsl4ToXsl(XSL4_In);
	settings.push_back(s);

	//-----
	s.m_function	= XSF_SendLatest;
	s.m_line		= xsl4ToXsl(XSL4_ReqData);
	// Disable polarity, offset and skip factor for ReqData
	s.m_polarity	= XSP_None;
	s.m_skipFactor	= 0;
	s.m_offset		= 0;
	settings.push_back(s);

	// Enable polarity, offset and skip factor for next settings
	s.m_polarity	= XSP_RisingEdge;
	s.m_skipFactor	= 1;
	s.m_offset		= 1;
	//-----

#if SUPPORT_XSL4_BiIn == 1
	s.m_line		= xsl4ToXsl(XSL4_BiIn);
	settings.push_back(s);
#endif

	s.m_function	= XSF_IntervalTransitionMeasurement;
	s.m_line		= xsl4ToXsl(XSL4_BiOut);
	s.m_pulseWidth	= 1;
	settings.push_back(s);

	s.m_function	= XSF_ClockBiasEstimation;
	s.m_line		= xsl4ToXsl(XSL4_ClockIn);
	s.m_offset		= 0;
	s.m_pulseWidth	= 0;
	s.m_clockPeriod = 1;
	settings.push_back(s);

	s.m_function	= XSF_StartSampling;
	s.m_line		= xsl4ToXsl(XSL4_In);
	s.m_skipFactor	= 0;
	s.m_clockPeriod = 0;
	s.m_pulseWidth	= 0;
	s.m_offset		= 1;
	settings.push_back(s);

	return settings;
}

/*! \brief get list of supported synchronizations settings for an Mti6x0Device */
XsSyncSettingArray supportedSyncSettingsForMt6x0Device()
{
	XsSyncSetting s;
	XsSyncSettingArray settings;

	//enable/disabled parameters per functions
	s.m_polarity = XSP_RisingEdge;
	s.m_offset = 0;
	s.m_skipFactor = 1;
	s.m_triggerOnce = 1;
	s.m_skipFirst = 1;
	s.m_pulseWidth = 0;
	s.m_clockPeriod = 0;

	s.m_function	= XSF_TriggerIndication;
	s.m_line		= XSL_In1;
	settings.push_back(s);
	s.m_line		= XSL_In2;
	settings.push_back(s);

	s.m_function	= XSF_SendLatest;
	s.m_line		= XSL_In1;
	settings.push_back(s);
	s.m_line		= XSL_In2;
	settings.push_back(s);

	//-----
	s.m_function	= XSF_SendLatest;
	s.m_line		= XSL_ReqData;
	// Disable polarity, offset and skip factor for ReqData
	s.m_polarity	= XSP_None;
	s.m_skipFactor	= 0;
	s.m_offset		= 0;
	s.m_skipFirst	= 0;
	s.m_triggerOnce = 0;
	settings.push_back(s);

	// Enable polarity, offset and skip factor for next settings
	s.m_polarity	= XSP_RisingEdge;
	s.m_skipFactor	= 1;
	s.m_offset		= 1;
	//-----

	s.m_function	= XSF_IntervalTransitionMeasurement;
	s.m_line		= XSL_Out1;
	s.m_pulseWidth	= 1;
	settings.push_back(s);
	s.m_skipFirst	= 0;
	s.m_triggerOnce = 0;

	s.m_function	= XSF_ClockBiasEstimation;
	s.m_offset		= 0;
	s.m_pulseWidth	= 0;
	s.m_clockPeriod = 1;
	s.m_line		= XSL_In1;
	settings.push_back(s);
	s.m_line		= XSL_In2;
	settings.push_back(s);

	s.m_function	= XSF_StartSampling;
	s.m_skipFactor	= 0;
	s.m_clockPeriod = 0;
	s.m_pulseWidth	= 0;
	s.m_offset		= 1;
	s.m_skipFirst	= 1;
	s.m_line		= XSL_In1;
	settings.push_back(s);
	s.m_line		= XSL_In2;
	settings.push_back(s);

	return settings;
}

/*! \brief get list of supported synchronizations settings for an Mti670Device */
XsSyncSettingArray supportedSyncSettingsForMti670Device()
{
	XsSyncSettingArray settings = supportedSyncSettingsForMt6x0Device();

	XsSyncSetting s;

	// add XSF_Gnss1Pps with gnss clock in setting
	s.m_function = XSF_Gnss1Pps;
	s.m_polarity = XSP_None;
	s.m_offset = 0;
	s.m_pulseWidth = 1;
	s.m_clockPeriod = 0;
	s.m_skipFactor = 0;
	s.m_line = XSL_In1;
	settings.push_back(s);
	s.m_line = XSL_In2;
	settings.push_back(s);

	return settings;
}

XsSyncSettingArray supportedSyncSettingsForMark5MtigX10Device()
{
	XsSyncSettingArray settings = supportedSyncSettingsForMtigDevice();

	XsSyncSetting timePulse1Pps(xsl4ToXsl(XSL4_BiOut), XSF_Gnss1Pps, XSP_None, 0, 0, 0, 0, 0, 0);
	settings.push_back(timePulse1Pps);

	return settings;
}

/*! \brief get list of supported synchronizations settings for an MtigDevice */
XsSyncSettingArray supportedSyncSettingsForMtigDevice()
{
	//extend ClockBiasEstimation to support GPS clock line
	XsSyncSettingArray settings = supportedSyncSettingsForMtiDevice();
	for (auto it = settings.begin(); it != settings.end(); ++it)
	{
		//insert a copy of existing XSF_ClockBiasEstimation with gps line setting
		if(it->m_function == XSF_ClockBiasEstimation)
		{
			XsSyncSetting s = *it;
			s.m_line = xsl4ToXsl(XSL4_GnssClockIn);
			// Disable all parameters
			s.m_polarity = XSP_None;
			s.m_pulseWidth = 0;
			s.m_offset = 0;
			s.m_skipFactor = 0;
			s.m_clockPeriod = 0;
			settings.insert(s, it);
			break;
		}
	}

	return settings;
}

/*! \brief get list of supported synchronizations settings for an Mti7Device */
XsSyncSettingArray supportedSyncSettingsForMti7Device()
{
	XsSyncSettingArray settings = supportedSyncSettingsForMtiXDevice();

	XsSyncSetting s;

	// add XSF_ClockBiasEstimation with clock line in setting
	s.m_function = XSF_ClockBiasEstimation;
	s.m_line = xsl4ToXsl(XSL4_In);
	s.m_offset = 0;
	s.m_pulseWidth = 0;
	s.m_clockPeriod = 1;
	s.m_skipFactor = 1;
	settings.push_back(s);

	// add XSF_ClockBiasEstimation with gps line in setting
	s.m_function = XSF_ClockBiasEstimation;
	s.m_line = xsl4ToXsl(XSL4_GnssClockIn);
	s.m_polarity = XSP_None;
	s.m_offset = 0;
	s.m_pulseWidth = 0;
	s.m_clockPeriod = 0;
	s.m_skipFactor = 0;
	settings.push_back(s);

	// add XSF_Gnss1Pps with gnss clock in setting
	s.m_function = XSF_Gnss1Pps;
	s.m_line = xsl4ToXsl(XSL4_Gnss1Pps);
	s.m_polarity = XSP_None;
	s.m_offset = 0;
	s.m_pulseWidth = 1;
	s.m_clockPeriod = 0;
	s.m_skipFactor = 0;
	settings.push_back(s);

	return settings;
}

/*! \brief get list of supported synchronizations settings for an MtDevice */
XsSyncSettingArray supportedSyncSettingsForMtDevice()
{
	XsSyncSetting s;
	XsSyncSettingArray settings;

	//enable/disabled options
	s.m_polarity = XSP_RisingEdge;
	s.m_skipFactor = 1;
	s.m_offset = 1;
	s.m_triggerOnce = 0;
	s.m_skipFirst = 0;
	s.m_pulseWidth = 0;
	s.m_clockPeriod = 0;

	s.m_function = XSF_TriggerIndication;
	s.m_line		= XSL_In1;
	settings.push_back(s);

	s.m_function = XSF_SendLatest;
	s.m_line		= XSL_In1;
	settings.push_back(s);

	s.m_function	= XSF_IntervalTransitionMeasurement;
	s.m_line		= XSL_Out1;
	s.m_pulseWidth = 1;
	settings.push_back(s);

	s.m_function = XSF_ClockBiasEstimation;
	s.m_line		= XSL_ClockIn;
	s.m_pulseWidth = 0;
	s.m_clockPeriod = 1;
	s.m_offset = 0;
	settings.push_back(s);


	return settings;
}

/*! \brief get list of supported synchronizations settings for MtDevice with GNSS */
XsSyncSettingArray supportedSyncSettingsForGnssDevice()
{
	XsSyncSetting s;
	XsSyncSettingArray settings = supportedSyncSettingsForMtDevice();

	s.m_function = XSF_ClockBiasEstimation;
	s.m_line		= XSL_GnssClockIn;
	// Disable all parameters
	s.m_polarity = XSP_None;
	s.m_pulseWidth = 0;
	s.m_offset = 0;
	s.m_skipFactor = 0;
	s.m_clockPeriod = 0;
	settings.push_back(s);

	return settings;
}

/*! \brief get list of supported synchronization settings for an MtiX Device */
XsSyncSettingArray supportedSyncSettingsForMtiXDevice()
{
	XsSyncSetting s;
	XsSyncSettingArray settings;

	//enable/disabled parameters per functions
	s.m_polarity = XSP_RisingEdge;
	s.m_offset = 1;
	s.m_skipFactor = 1;
	s.m_triggerOnce = 0;
	s.m_skipFirst = 0;
	s.m_pulseWidth = 0;
	s.m_clockPeriod = 0;

	s.m_function	= XSF_SendLatest;
	s.m_line		= xslgmtToXsl(XSLGMT_In1);
	settings.push_back(s);

	//-----
	s.m_function	= XSF_SendLatest;
	s.m_line		= xslgmtToXsl(XSLGMT_ReqData);
	// Disable polarity, offset and skip factor for ReqData
	s.m_polarity	= XSP_None;
	s.m_skipFactor	= 0;
	s.m_offset		= 0;
	settings.push_back(s);

	return settings;
}

/*! \brief get list of supported synchronizations settings for an Awinda base station */
XsSyncSettingArray supportedSyncSettingsForAwindaBaseStation()
{
	XsSyncSetting s;
	XsSyncSettingArray settings;

	// ----- Input triggers -----
	std::set<XsSyncFunction> inputSyncFunctions;
	inputSyncFunctions.insert(XSF_StartRecordingIn);
	inputSyncFunctions.insert(XSF_StopRecordingIn);
	inputSyncFunctions.insert(XSF_ResetTimer);
	inputSyncFunctions.insert(XSF_TriggerIndication);

	for (XsSyncFunction const & syncFunction : inputSyncFunctions)
	{
		s.m_line = XSL_In1;
		s.m_polarity = XSP_RisingEdge;
		s.m_function = syncFunction;
		s.m_pulseWidth = 0;
		s.m_offset = 0;
		s.m_skipFirst = 1;
		s.m_skipFactor = 1;
		s.m_clockPeriod = 0;
		s.m_triggerOnce = 1;

		settings.push_back(s);

		s.m_line = XSL_In2;
		s.m_polarity = XSP_RisingEdge;
		s.m_function = syncFunction;
		s.m_pulseWidth = 0;
		s.m_offset = 0;
		s.m_skipFirst = 1;
		s.m_skipFactor = 1;
		s.m_clockPeriod = 0;
		s.m_triggerOnce = 1;

		settings.push_back(s);
	}

	// ----- Output triggers -----
	std::set<XsSyncFunction> outputSyncFunctions;
	outputSyncFunctions.insert(XSF_StartRecordingOut);
	outputSyncFunctions.insert(XSF_StopRecordingOut);
	outputSyncFunctions.insert(XSF_GotoOperational);
	outputSyncFunctions.insert(XSF_IntervalTransitionMeasurement);
	outputSyncFunctions.insert(XSF_IntervalTransitionRecording);
	for (XsSyncFunction const & syncFunction : outputSyncFunctions)
	{
		s.m_line = XSL_Out1;
		s.m_polarity = XSP_RisingEdge;
		s.m_function = syncFunction;
		s.m_pulseWidth = 1;
		s.m_offset = 0;
		s.m_skipFirst = 1;
		s.m_skipFactor = 1;
		s.m_clockPeriod = 0;
		s.m_triggerOnce = 1;

		settings.push_back(s);

		s.m_line = XSL_Out2;
		s.m_polarity = XSP_RisingEdge;
		s.m_function = syncFunction;
		s.m_pulseWidth = 1;
		s.m_offset = 0;
		s.m_skipFirst = 1;
		s.m_skipFactor = 1;
		s.m_clockPeriod = 0;
		s.m_triggerOnce = 1;

		settings.push_back(s);
	}

	return settings;
}

/*! \returns true if the awinda settings \a setting1 and \a setting2 are compatible
	\note This *very* specific for awinda
	Basically this checks that if a specific function is configured for In1 and In2 (or Out1 and Out2), then the other settings of both should match.
	This is because this get combined into one function with the line: both setting
*/
bool isAwindaSettingCompatible(XsSyncSetting const & setting1, XsSyncSetting const & setting2)
{
	bool isCompatible = true;
	if (setting1.m_function == setting2.m_function)
	{
		bool areInputs = (setting1.m_line == XSL_In1 || setting1.m_line == XSL_In2) && (setting2.m_line == XSL_In1 || setting2.m_line == XSL_In2);
		bool areOutputs = (setting1.m_line == XSL_Out1 || setting1.m_line == XSL_Out2) && (setting2.m_line == XSL_Out1 || setting2.m_line == XSL_Out2);
		if (areInputs || areOutputs)
		{
			if (setting1.m_line != setting2.m_line)
			{
				if (setting1.m_polarity != setting2.m_polarity ||
					setting1.m_pulseWidth != setting2.m_pulseWidth ||
					setting1.m_offset != setting2.m_offset ||
					setting1.m_skipFirst != setting2.m_skipFirst ||
					setting1.m_skipFactor != setting2.m_skipFactor ||
					setting1.m_clockPeriod != setting2.m_clockPeriod ||
					setting1.m_triggerOnce != setting2.m_triggerOnce)
				{
					isCompatible = false;
				}
			}
		}
	}
	return isCompatible;
}

/*! \returns true if the sync station settings \a setting1 and \a setting2 are compatible
	\note This *very* specific for a sync station
	\sa isAwindaSettingCompatible
*/
bool isSyncStationSettingCompatible(XsSyncSetting const & setting1, XsSyncSetting const & setting2)
{
	return isAwindaSettingCompatible(setting1, setting2);
}

/*! \returns the time resolution in microseconds for an awinda device
*/
unsigned int awindaTimeResolutionInMicroseconds()
{
	return 1000;
}

/*! \returns the time resolution in microseconds for an sync station device
*/
unsigned int syncStationTimeResolutionInMicroseconds()
{
	return awindaTimeResolutionInMicroseconds();;
}

/*! \returns the time resolution in microseconds for an Mtmk4 device
*/
unsigned int mtiTimeResolutionInMicroseconds()
{
	return 100;
}

/*! \returns the time resolution in microseconds for an Mtx2 device
*/
unsigned int mtx2TimeResolutionInMicroseconds()
{
	return 100;
}

} // End namespace
