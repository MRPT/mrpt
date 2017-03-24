/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*! \file Cmt3.cpp

	For information about objects in this file, see the appropriate header:
	\ref Cmt3.h

	\section FileCopyright Copyright Notice 
	Copyright (C) Xsens Technologies B.V., 2006.  All rights reserved.
	
	This source code is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.
	
	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
	
	\section FileChangelog	Changelog
	\par 2006-04-28, v0.0.1
	\li Job Mulder:	Created
	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/

#include "cmt3.h"
#include <math.h>
#include "xsens_janitors.h"

#ifdef _LOG_CMT3
#	define CMT3LOG		CMTLOG
#	define CMT3EXITLOG	JanitorLogFunc<XsensResultValue,uint32_t> _cmtExitLog(CMTLOG,"L3: " __FUNCTION__ " returns %u\n",m_lastResult);
#else
#	define CMT3LOG(...)
#	define CMT3EXITLOG
#endif

#ifdef _LOG_CMT3_DATA
#	define CMT3LOGDAT		CMTLOG
#	define CMT3EXITLOGDAT	JanitorLogFunc<XsensResultValue,uint32_t> _cmtExitLog(CMTLOG,"L3: " __FUNCTION__ " returns %u\n",m_lastResult);
#else
#	define CMT3LOGDAT(...)
#	define CMT3EXITLOGDAT
#endif

#define CMT3F_DEVINFO_SIZE		(sizeof(CmtDeviceId) + sizeof(CmtDataFormat))

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// Support  classes ////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

void CmtDeviceConfiguration::readFromMessage(const void* message)
{
	xsens::Message msg((const uint8_t*) message,0);

	m_masterDeviceId = msg.getDataLong(0);
	m_samplingPeriod = msg.getDataShort(4);
	m_outputSkipFactor = msg.getDataShort(6);
	m_syncinMode = msg.getDataShort(8);
	m_syncinSkipFactor = msg.getDataShort(10);
	m_syncinOffset = msg.getDataLong(12);
	memcpy(m_date,msg.getDataBuffer(16),8);
	memcpy(m_time,msg.getDataBuffer(24),8);
	memcpy(m_reservedForHost,msg.getDataBuffer(32),32);
	memcpy(m_reservedForClient,msg.getDataBuffer(64),32);
	m_numberOfDevices = msg.getDataShort(96);

	for (uint16_t i = 0; i < m_numberOfDevices; ++i)
	{
		m_deviceInfo[i].m_deviceId = msg.getDataLong(98+i*20);
		m_deviceInfo[i].m_dataLength = msg.getDataShort(102+i*20);
		m_deviceInfo[i].m_outputMode = msg.getDataShort(104+i*20);
		m_deviceInfo[i].m_outputSettings = msg.getDataLong(106+i*20);
		memcpy(m_deviceInfo[i].m_reserved,msg.getDataBuffer(110+i*20),8);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Compute the period and skip factor.
void CmtDeviceMode::getPeriodAndSkipFactor(uint16_t& period,uint16_t& skip) const
{
	if (m_sampleFrequency == 0)
	{
		period = 0;
		skip = 0;
		return;
	}
	if (m_sampleFrequency >= 512)
	{
		period = 225;
		skip = 0;
		return;
	}

	int32_t freq, sf = m_sampleFrequency;

	// first try the simple ones
	skip = 0;
	freq = sf;
	while (freq < 100)
	{
		++skip;
		freq += sf;
	}

	period = (uint16_t) (115200 / freq);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the real sample frequency in Hz
double CmtDeviceMode::getRealSampleFrequency(void) const
{
	uint16_t skip, period;
	getPeriodAndSkipFactor(period,skip);
	return (115200.0 / ((1.0 + (double) skip) * (double) period));
}

//////////////////////////////////////////////////////////////////////////////////////////
// Compute sample frequency from the period and skip factor.
void CmtDeviceMode::setPeriodAndSkipFactor(uint16_t period, uint16_t skip)
{
	m_sampleFrequency = (uint16_t) floor((115200.0 / ((1.0 + (double) skip) * (double) period))+0.5);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Compute sample frequency from the period and skip factor.
bool CmtDeviceMode::operator == (const CmtDeviceMode& dev) const
{
	return m_outputMode == dev.m_outputMode && m_outputSettings == dev.m_outputSettings && m_sampleFrequency == dev.m_sampleFrequency;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the real sample frequency in Hz
double CmtDeviceMode2::getRealSampleFrequency(void) const
{
	if (m_skip != 0xFFFF)
		return (115200.0 / ((1.0 + (double) m_skip) * (double) m_period));
	else
		return (115200.0 / ((double) m_period));
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the sample frequency in Hz
uint16_t CmtDeviceMode2::getSampleFrequency(void) const
{
	if (m_skip != 0xFFFF)
		return (uint16_t) floor((115200.0 / ((1.0 + (double) m_skip) * (double) m_period))+0.5);
	else
		return (uint16_t) floor((115200.0 / ((double) m_period))+0.5);
}

void CmtDeviceMode2::setSampleFrequency(uint16_t frequency)
{
	if (frequency == 0)
	{
		m_period = 0;
		m_skip = 0;
		return;
	}
	if (frequency >= 512)
	{
		m_period = 225;
		m_skip = 0;
		return;
	}

	int32_t freq, sf = frequency;

	// first try the simple ones
	m_skip = 0;
	freq = sf;
	while (freq < 100)
	{
		++m_skip;
		freq += sf;
	}

	m_period = (uint16_t) (115200 / freq);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Compute sample frequency from the period and skip factor.
bool CmtDeviceMode2::operator == (const CmtDeviceMode2& dev) const
{
	return m_outputMode == dev.m_outputMode && m_outputSettings == dev.m_outputSettings && m_period == dev.m_period && m_skip == dev.m_skip;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Common part of a data request function
#define DO_DATA_REQUEST_BID(req,bid)\
	Message snd(req,0);\
	Message rcv;\
	if (bid == CMT_BID_INVALID || bid == CMT_BID_BROADCAST)\
		return (m_lastResult = XRV_INVALIDID);\
	if (!m_readFromFile)\
	{\
		snd.setBusId(bid);\
		m_serial.writeMessage(&snd);\
		if ((m_lastResult = m_serial.waitForMessage(&rcv,req+1,0,true)) != XRV_OK)\
			return m_lastResult;\
		if (m_logging)\
			m_logFile.writeMessage(&rcv);\
		if (rcv.getMessageId() == CMT_MID_ERROR)\
		{	m_lastHwErrorDeviceId = m_config.m_masterDeviceId;\
			if (rcv.getDataSize() >= 2)\
			{\
				uint8_t biddy = rcv.getDataByte(1);\
				getDeviceId(biddy,m_lastHwErrorDeviceId);\
			}\
			return m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte();\
		}\
	}\
	else\
	while (1)\
	{\
		if ((m_lastResult = m_logFile.readMessage(&rcv,req+1)) != XRV_OK)\
			return m_lastResult;\
		if (rcv.getBusId() == bid || (rcv.getBusId() == 1 && bid == CMT_BID_MASTER))\
			break;\
	}

#define DO_DATA_REQUEST(req)\
	uint8_t bid = getBusIdInternal(deviceId);\
	DO_DATA_REQUEST_BID(req,bid);

//////////////////////////////////////////////////////////////////////////////////////////
// Common part of a serial data set function
#define DO_DATA_SET_BID(req,size,type,data,bid)\
	Message	snd(req,size);\
	Message rcv;\
	snd.setData ## type (data);\
	if (bid == CMT_BID_BROADCAST)\
	{\
		for (uint8_t i=0;i<m_config.m_numberOfDevices;++i)\
		{\
			snd.setBusId(i+1);\
			m_serial.writeMessage(&snd);\
			m_lastResult = m_serial.waitForMessage(&rcv,req+1,0,true);\
			if (m_lastResult != XRV_OK)\
				return m_lastResult;\
			if (m_logging)\
				m_logFile.writeMessage(&rcv);\
			if (rcv.getMessageId() == CMT_MID_ERROR)\
			{	m_lastHwErrorDeviceId = m_config.m_masterDeviceId;\
				if (rcv.getDataSize() >= 2)\
				{\
					uint8_t biddy = rcv.getDataByte(1);\
					getDeviceId(biddy,m_lastHwErrorDeviceId);\
				}\
				return m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte();\
			}\
		}\
	} else {\
		snd.setBusId(bid);\
		m_serial.writeMessage(&snd);\
		m_lastResult = m_serial.waitForMessage(&rcv,req+1,0,true);\
		if (m_lastResult != XRV_OK)\
			return m_lastResult;\
		if (m_logging)\
			m_logFile.writeMessage(&rcv);\
		if (rcv.getMessageId() == CMT_MID_ERROR)\
		{	m_lastHwErrorDeviceId = m_config.m_masterDeviceId;\
			if (rcv.getDataSize() >= 2)\
			{\
				uint8_t biddy = rcv.getDataByte(1);\
				getDeviceId(biddy,m_lastHwErrorDeviceId);\
			}\
			return m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte();\
		}\
	}

#define DO_DATA_SET(req,size,type,data)\
	uint8_t bid = getBusIdInternal(deviceId);\
	if (bid == CMT_BID_INVALID)\
		return (m_lastResult = XRV_INVALIDID);\
	DO_DATA_SET_BID(req,size,type,data,bid);

#define HANDLE_ERR_RESULT\
	if (rcv.getMessageId() == CMT_MID_ERROR)\
	{\
		m_lastHwErrorDeviceId = m_config.m_masterDeviceId;\
		if (rcv.getDataSize() >= 2)\
		{\
			uint8_t biddy = rcv.getDataByte(1);\
			getDeviceId(biddy,m_lastHwErrorDeviceId);\
		}\
		return m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte(0);\
	}


//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt3  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

namespace xsens {

//////////////////////////////////////////////////////////////////////////////////////////
// Default constructor, initializes all members to their default values.
Cmt3::Cmt3()
{
	m_lastResult = XRV_OK;
	m_rtcInitialized = false;
	m_useRtc = true;
	m_measuring = false;
	m_timeoutConf = CMT3_DEFAULT_TIMEOUT_CONF;
	m_timeoutMeas = CMT3_DEFAULT_TIMEOUT_MEAS;
	m_gotoConfigTries = CMT_GOTO_CONFIG_TRIES;
	m_readFromFile = false;
	//m_sampleFrequency = 100.0;
	m_period = CMT_DEFAULT_PERIOD;
	m_skip = CMT_DEFAULT_SKIP;
	memset(m_eMtsData,0,sizeof(m_eMtsData));
	memset(&m_config,0,sizeof(m_config));
	m_logging = false;

	clearHwError();
}

//////////////////////////////////////////////////////////////////////////////////////////
// Destructor, de-initializes, frees memory allocated for buffers, etc.
Cmt3::~Cmt3()
{
	m_serial.close();
	m_logFile.close();
	for (uint32_t i = 0; i < CMT_MAX_DEVICES_PER_PORT; ++i)
	{
		CHKFREENUL(m_eMtsData[i]);
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Close the serial communication port.
XsensResultValue Cmt3::closePort(bool gotoConfigFirst)
{
	CMT3LOG("L3: closePort\n");
	CMT3EXITLOG;

	if (m_measuring && gotoConfigFirst)
		gotoConfig();
	CMT3LOG("L3: Closing L2 port\n");
	m_serial.close();
	m_measuring = false;
	if (m_logFile.isOpen())
	{
		m_readFromFile = true;
		m_logging = false;
	}
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Cmt3::fillRtc(Packet* pack)
{
	if (!m_rtcInitialized)
	{
		m_rtcStart = pack->m_toa;
		m_rtcLastSc = pack->getSampleCounter();
		m_rtcCount = m_rtcLastSc;
		CmtDeviceMode2 mode;
		mode.m_period = m_period;
		mode.m_skip = m_skip;
		m_rtcMsPerSample = 1000.0 / mode.getRealSampleFrequency();
		pack->m_rtc = m_rtcStart;
		m_rtcInitialized = true;
	}
	else
	{
		CmtMtTimeStamp sc = pack->getSampleCounter();
		CmtMtTimeStamp scdiff = sc - m_rtcLastSc;
		m_rtcLastSc = sc;
		m_rtcCount += scdiff;
		pack->m_rtc = m_rtcStart + (TimeStamp) floor(m_rtcMsPerSample * (double) m_rtcCount + 0.5);
	}
}

#if 0
obsolete:
//////////////////////////////////////////////////////////////////////////////////////////
// Get the state (enabled/disabled) of the AMD algorithm
XsensResultValue Cmt3::getAmdState(bool& state, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getAmdState %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQAMD);
	state = (rcv.getDataShort() != 0);
	return m_lastResult = XRV_OK;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Get the XM batery level
XsensResultValue Cmt3::getBatteryLevel(uint8_t& level)
{
	CMT3LOG("L3: getBatteryLevel\n");
	CMT3EXITLOG;

	DO_DATA_REQUEST_BID(CMT_MID_REQBATLEVEL,CMT_BID_MASTER);
	level = rcv.getDataByte();
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Get the baudrate that is currently being used by the port
XsensResultValue Cmt3::getBaudrate(uint32_t& baudrate)
{
	CMT3LOG("L3: getBaudrate\n");
	CMT3EXITLOG;

	if (!m_serial.isOpen())
		return m_lastResult = XRV_NOPORTOPEN;
	baudrate = m_baudrate;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Get the state of the bluetooth communication, on (true) or off (false)
XsensResultValue Cmt3::getBluetoothState(bool& enabled)
{
	CMT3LOG("L3: getBluetoothState\n");
	CMT3EXITLOG;

	DO_DATA_REQUEST_BID(CMT_MID_REQBTDISABLE,CMT_BID_MASTER);
	enabled = (rcv.getDataByte() == 0);	// invert the result as it is a disable field
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the BusId of a device.
XsensResultValue Cmt3::getBusId (uint8_t& busId, const CmtDeviceId deviceId) const
{
	CMT3LOG("L3: getBusId %08x\n",deviceId);
	CMT3EXITLOG;

	if (deviceId == CMT_DID_MASTER || deviceId == m_config.m_masterDeviceId)
	{
		busId = CMT_BID_MASTER;
		return m_lastResult = XRV_OK;
	}

	for (uint16_t i=0 ; i <= m_config.m_numberOfDevices ; ++i)
		if (m_config.m_deviceInfo[i].m_deviceId == deviceId)
		{
			busId = i+1;
			return m_lastResult = XRV_OK;
		}
	return m_lastResult = XRV_NOTFOUND;
}

//////////////////////////////////////////////////////////////////////////////////////////
uint8_t Cmt3::getBusIdInternal(const CmtDeviceId devId) const
{
	if (devId == CMT_DID_MASTER)
		return CMT_BID_MASTER;

	if (devId == CMT_DID_BROADCAST)
		return CMT_BID_BROADCAST;

	if (m_config.m_masterDeviceId == devId)
		return CMT_BID_MASTER;

	for (uint16_t i = 0;i <= m_config.m_numberOfDevices;++i)
	{
		if (m_config.m_deviceInfo[i].m_deviceId == devId)
			return (uint8_t) (i+1);
	}
	return CMT_BID_INVALID;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the XM bus power state.
XsensResultValue Cmt3::getBusPowerState(bool& enabled)
{
	CMT3LOG("L3: getBusPowerState\n");
	CMT3EXITLOG;

	DO_DATA_REQUEST_BID(CMT_MID_BUSPWR,CMT_BID_MASTER);
	enabled = (rcv.getDataShort() != 0);	// invert the result as it is a disable field
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
Cmt2f* Cmt3::getCmt2f(void)
{
	return &m_logFile;
}

//////////////////////////////////////////////////////////////////////////////////////////
Cmt2s* Cmt3::getCmt2s(void)
{
	return &m_serial;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the complete device configuration of a device.
XsensResultValue Cmt3::getConfiguration(CmtDeviceConfiguration& configuration)
{
	CMT3LOG("L3: getConfiguration\n");
	CMT3EXITLOG;

	if (!(m_serial.isOpen() || m_logFile.isOpen()))
		return m_lastResult = XRV_INVALIDOPERATION;
	
	memcpy(&configuration,&m_config,sizeof(CmtDeviceConfiguration));

	if (m_logging)
	{
		// fake the message receipt
		Message msg(CMT_MID_CONFIGURATION,98 + CMT_CONF_BLOCKLEN*m_config.m_numberOfDevices);
		msg.setBusId(CMT_BID_MASTER);
		
		msg.setDataLong(m_config.m_masterDeviceId, 0);
		msg.setDataShort(m_config.m_samplingPeriod, 4);
		msg.setDataShort(m_config.m_outputSkipFactor, 6);
		msg.setDataShort(m_config.m_syncinMode, 8);
		msg.setDataShort(m_config.m_syncinSkipFactor, 10);
		msg.setDataLong(m_config.m_syncinOffset, 12);
		memcpy(msg.getDataBuffer(16), m_config.m_date,8);
		memcpy(msg.getDataBuffer(24), m_config.m_time, 8);
		memcpy(msg.getDataBuffer(32), m_config.m_reservedForHost,32);
		memcpy(msg.getDataBuffer(64), m_config.m_reservedForClient,32);
		msg.setDataShort(m_config.m_numberOfDevices, 96);

		for (uint16_t i = 0; i < m_config.m_numberOfDevices; ++i)
		{
			msg.setDataLong(m_config.m_deviceInfo[i].m_deviceId, 98+i*20);
			msg.setDataShort(m_config.m_deviceInfo[i].m_dataLength, 102+i*20);
			msg.setDataShort(m_config.m_deviceInfo[i].m_outputMode, 104+i*20);
			msg.setDataLong(m_config.m_deviceInfo[i].m_outputSettings, 106+i*20);
			memcpy(msg.getDataBuffer(110+i*20), m_config.m_deviceInfo[i].m_reserved,8);
		}

		msg.recomputeChecksum();

		writeMessageToLogFile(msg);
	}
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the number of bytes that are in a data message as sent by the device.
XsensResultValue Cmt3::getDataLength(uint32_t& length, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getDataLength %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQDATALENGTH);
	length = rcv.getDataShort();	// the sensor returns this value as a int16_t
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the number of connected devices. Returns 0 if not connected.
uint32_t Cmt3::getDeviceCount (void) const
{
	CMT3LOG("L3: getDeviceCount\n");
	if (m_serial.isOpen() || m_logFile.isOpen())
	{
		if (isXm())
			return m_config.m_numberOfDevices + 1;
		else
			return m_config.m_numberOfDevices;
	}
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the CmtDeviceId of the device at the given BusId 
XsensResultValue Cmt3::getDeviceId(const uint8_t busId, CmtDeviceId& deviceId) const
{
	CMT3LOG("L3: getDeviceId %u\n",(uint32_t) busId);
	CMT3EXITLOG;

	if (busId == CMT_BID_MASTER || busId == 0)
		deviceId = m_config.m_masterDeviceId;
	else
	{
		if (busId > m_config.m_numberOfDevices)
			return m_lastResult = XRV_INVALIDID;
		deviceId = m_config.m_deviceInfo[busId-1].m_deviceId;
	}
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the complete device output mode of a device.
XsensResultValue Cmt3::getDeviceMode(CmtDeviceMode& mode, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getDeviceMode %08x\n",deviceId);
	CMT3EXITLOG;

	CmtDeviceMode2 mode2;
	XsensResultValue rv;
	rv = getDeviceMode2(mode2, deviceId);
	if (rv == XRV_OK) {
		mode.m_outputMode = mode2.m_outputMode;
		mode.m_outputSettings = mode2.m_outputSettings;
		mode.m_sampleFrequency = mode2.getSampleFrequency();
	}
	return rv;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the complete device output mode of a device.
XsensResultValue Cmt3::getDeviceMode2(CmtDeviceMode2& mode, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getDeviceMode2 %08x\n",deviceId);
	CMT3EXITLOG;

	uint8_t bid = getBusIdInternal(deviceId);
	if (bid == CMT_BID_INVALID)
		return m_lastResult = XRV_INVALIDID;
	if (bid == 0)
		return m_lastResult = XRV_INVALIDID;
	if (bid == CMT_BID_MASTER)
		bid = 1;

	mode.m_period = m_period;
	mode.m_skip = m_skip;
	//mode.m_sampleFrequency = (uint16_t) floor(m_sampleFrequency+0.5);
	mode.m_outputMode = m_config.m_deviceInfo[bid-1].m_outputMode;
	mode.m_outputSettings = m_config.m_deviceInfo[bid-1].m_outputSettings;
	
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the eMts data of the specified sensor(s)
XsensResultValue Cmt3::getEMtsData(void* buffer, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getEMtsData %p %08x\n",buffer,deviceId);
	CMT3EXITLOG;

	uint8_t bid = getBusIdInternal(deviceId);
	if (bid == CMT_BID_INVALID)
		return m_lastResult = XRV_INVALIDID;
	if (!m_readFromFile && !m_serial.isOpen())
		return m_lastResult = XRV_NOPORTOPEN;
	if (m_readFromFile && !m_logFile.isOpen())
		return m_lastResult = XRV_NOFILEOPEN;
	if (buffer == NULL)
		return m_lastResult = XRV_NULLPTR;
		
	if (bid == CMT_BID_BROADCAST)
	{
		memset(buffer,0,m_config.m_numberOfDevices * CMT_EMTS_SIZE);
		uint8_t* buf = (uint8_t*) buffer;
		// loop over devices and request emts data for all of them
		for (uint32_t dev=0; dev < m_config.m_numberOfDevices; ++dev)
			if (getEMtsData(buf+dev*CMT_EMTS_SIZE,m_config.m_deviceInfo[dev].m_deviceId) != XRV_OK)
				return m_lastResult;
		//CMT3LOG("L3: getEMtsData (%08x) returns %d\n",deviceId,(int32_t)m_lastResult);
		return m_lastResult;
	}

	uint8_t dataIndex;

	if (isXm())
	{
		if (bid == CMT_BID_MASTER)
		{
			//CMT3LOG("L3: getEMtsData (%08x) returns XRV_INVALIDID\n",deviceId);
			return m_lastResult = XRV_INVALIDID;
		}
		dataIndex = bid-1;
	}
	else
		dataIndex = 0;		// when we have a single MT, we should use bank 0

	if (m_eMtsData[dataIndex] == NULL)
	{
		m_eMtsData[dataIndex] = malloc(CMT_EMTS_SIZE);

		// Xbus requires small data packets
#if 0
		if (isXm())
		{
			Message msg(CMT_MID_REQEMTS,3);
			Message rcv;
			uint8_t* tme = (uint8_t*) m_eMtsData[dataIndex];
			memset(tme,0,CMT_EMTS_SIZE);	// clear eMTS data
			msg.setBusId(bid);
			msg.setDataByte(0,0);			// read bank 0
			uint16_t emtPos = 0;
			for (uint8_t page=0;page < 4;++page)	// read all 4 pages, one at a time
			{
				msg.setDataByte(page,1);
				for (uint8_t part=0;part < 4;++part, ++emtPos)
				{
					if (!m_readFromFile)
					{
						msg.setDataByte(part,2);
						m_serial.writeMessage(&msg);
						m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_EMTSDATA,0,false);
					}
					else
						m_lastResult = m_logFile.readMessage(&rcv,CMT_MID_EMTSDATA);

					if (m_lastResult != XRV_OK)
					{
						FREENUL(m_eMtsData[dataIndex]);
						//CMT3LOG("L3: getEMtsData (%08x) returns %d\n",deviceId,(int32_t)m_lastResult);
						return m_lastResult;
					}
					if (!m_readFromFile && m_logging)
						m_logFile.writeMessage(&rcv);
					if (rcv.getMessageId() == CMT_MID_ERROR)
					{
						m_lastHwErrorDeviceId = m_config.m_masterDeviceId;
						if (rcv.getDataSize() >= 2)
						{
							uint8_t biddy = rcv.getDataByte(1);
							getDeviceId(biddy,m_lastHwErrorDeviceId);
						}
						m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte(0);
						FREENUL(m_eMtsData[dataIndex]);
						//CMT3LOG("L3: getEMtsData (%08x) returns %d\n",deviceId,(int32_t)m_lastResult);
						return m_lastResult;
					}

					memcpy(tme+(emtPos*66),rcv.getDataBuffer(),66);
				}
			}
		}	// if (isXm())
		else	// single MT, ask in one packet
		{
#endif
			Message msg(CMT_MID_REQEMTS,2);
			Message rcv;
			memset(m_eMtsData[dataIndex],0,CMT_EMTS_SIZE);	// clear eMTS data
			msg.setBusId(bid);
			msg.setDataByte(0,0);	// read bank 0
			msg.setDataByte(255,1);	// read all pages

			if (!m_readFromFile)
			{
				m_serial.writeMessage(&msg);
				m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_EMTSDATA,0,false);
			}
			else
				m_lastResult = m_logFile.readMessage(&rcv,CMT_MID_EMTSDATA);
			if (m_lastResult != XRV_OK)
			{
				FREENUL(m_eMtsData[dataIndex]);
				//CMT3LOG("L3: getEMtsData (%08x) returns %d\n",deviceId,(int32_t)m_lastResult);
				return m_lastResult;
			}
			if (!m_readFromFile && m_logging)
				m_logFile.writeMessage(&rcv);
			if (rcv.getMessageId() == CMT_MID_ERROR)
			{
				m_lastHwErrorDeviceId = m_config.m_masterDeviceId;
				if (rcv.getDataSize() >= 2)
				{
					uint8_t biddy = rcv.getDataByte(1);
					getDeviceId(biddy,m_lastHwErrorDeviceId);
				}
				m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte(0);
				FREENUL(m_eMtsData[dataIndex]);
				//CMT3LOG("L3: getEMtsData (%08x) returns %d\n",deviceId,(int32_t)m_lastResult);
				return m_lastResult;
			}

			memcpy(m_eMtsData[dataIndex],rcv.getDataBuffer(),CMT_EMTS_SIZE);
#if 0
		}
#endif
	} else if (m_logging)
	{
		// fake the message receipt
		Message msg(CMT_MID_EMTSDATA,CMT_EMTS_SIZE);
		msg.setBusId(bid);
		msg.setDataBuffer((const uint8_t*) m_eMtsData[dataIndex],0,CMT_EMTS_SIZE);
		writeMessageToLogFile(msg);
	}

	memcpy(buffer,m_eMtsData[dataIndex],CMT_EMTS_SIZE);
	//CMT3LOG("L3: getEMtsData (%08x) returns XRV_OK\n",deviceId);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the error mode
XsensResultValue Cmt3::getErrorMode(uint16_t& mode, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getErrorMode %08x\n",deviceId);
	CMT3EXITLOG;

	if (isXm())
	{
		DO_DATA_REQUEST_BID(CMT_MID_REQXMERRORMODE,CMT_BID_MASTER);
		mode = rcv.getDataShort();
	}
	else
	{
		DO_DATA_REQUEST(CMT_MID_REQERRORMODE);
		mode = rcv.getDataShort();
	}
	return m_lastResult = XRV_OK;
}

#if 0
obsolete:
//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the filter settings of a device.
XsensResultValue Cmt3::getFilterSettings(CmtFilterSettings& settings, const CmtDeviceId deviceId)
{
	Message snd;
	Message rcv;
	uint8_t bid = getBusIdInternal(deviceId);
	if (bid == CMT_BID_INVALID)
		return (m_lastResult = XRV_INVALIDID);
	
	bool xm = isXm();

	if (xm && deviceId == m_config.m_masterDeviceId)
		return m_lastResult = XRV_INVALIDOPERATION;

	if (!xm)
		bid = CMT_BID_MASTER;

	// first setting: gain
	snd.setBusId(bid);
	snd.setDataByte(0);
	snd.setMessageId(CMT_MID_REQFILTERSETTINGS);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQFILTERSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_gain = rcv.getDataFloat(1);

	// second setting: weighting
	snd.setBusId(bid);
	snd.setDataByte(1);
	snd.setMessageId(CMT_MID_REQFILTERSETTINGS);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQFILTERSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_weighting = rcv.getDataFloat(1);

	return m_lastResult = XRV_OK;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the firmware revision of a device.
XsensResultValue Cmt3::getFirmwareRevision(CmtVersion& revision, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getFirmwareRevision %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQFWREV);

	revision.m_major	= rcv.getDataByte(0);
	revision.m_minor	= rcv.getDataByte(1);
	revision.m_revision	= rcv.getDataByte(2);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the heading offset of a device. The range is -pi to +pi.
XsensResultValue Cmt3::getHeading(double& heading, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getHeading %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQHEADING);
	heading = rcv.getDataFloat();
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the location ID of a device. The buffer should be at least 20 bytes.
XsensResultValue Cmt3::getLocationId(uint16_t& locationId, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getLocationId %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQLOCATIONID);
	locationId = rcv.getDataShort();
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the current log file read position
XsensResultValue Cmt3::getLogFileReadPosition(CmtFilePos& pos)
{
	CMT3LOG("L3: getLogFileReadPosition\n");
	CMT3EXITLOG;

	if (m_logFile.isOpen())
	{
		pos = m_logFile.getReadPosition();
		return m_lastResult = XRV_OK;
	}
	pos = 0;
	return m_lastResult = XRV_NOFILEOPEN;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the size of the log file
XsensResultValue Cmt3::getLogFileSize(CmtFilePos& size)
{
	CMT3LOG("L3: getLogFileSize\n");
	CMT3EXITLOG;

	if (m_logFile.isOpen())
	{
		size = m_logFile.getFileSize();
		return m_lastResult = XRV_OK;
	}
	size = 0;
	return m_lastResult = XRV_NOFILEOPEN;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the name of the open log file or an empty string if no logfile is open
XsensResultValue Cmt3::getLogFileName(char* filename)
{
	CMT3LOG("L3: getLogFileName (char)\n");
	CMT3EXITLOG;

	if (m_logFile.isOpen())
		return m_lastResult = m_logFile.getName(filename);
	filename[0] = '\0';
	return m_lastResult = XRV_NOFILEOPEN;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the name of the open log file or an empty string if no logfile is open
XsensResultValue Cmt3::getLogFileName(wchar_t* filename)
{
	CMT3LOG("L3: getLogFileName (wchar_t)\n");
	CMT3EXITLOG;

	if (m_logFile.isOpen())
		return m_lastResult = m_logFile.getName(filename);
	filename[0] = L'\0';
	return m_lastResult = XRV_NOFILEOPEN;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the magnetic declination. The range is -pi to +pi.
XsensResultValue Cmt3::getMagneticDeclination(double& declination, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getMagneticDeclination %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQMAGNETICDECLINATION);
	declination = rcv.getDataFloat();
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the device Id of the first device (master)
CmtDeviceId Cmt3::getMasterId(void)
{
	CMT3LOG("L3: getMasterId\n");

	if (m_serial.isOpen() || m_logFile.isOpen())
		return m_config.m_masterDeviceId;
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the nr of connected MTs (excludes XMs)
uint16_t Cmt3::getMtCount(void) const
{
	CMT3LOGDAT("L3: getMtCount\n");

	if (m_serial.isOpen() || m_logFile.isOpen())
		return m_config.m_numberOfDevices;
	return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the CmtDeviceId of the MT device with the given index
XsensResultValue Cmt3::getMtDeviceId (const uint8_t index, CmtDeviceId& deviceId) const
{
	CMT3LOG("L3: getMtDeviceId %u\n",(uint32_t) index);
	CMT3EXITLOG;

	if (index >= m_config.m_numberOfDevices)
		return m_lastResult = XRV_INVALIDPARAM;
	deviceId = m_config.m_deviceInfo[index].m_deviceId;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the port that the object is connected to.
XsensResultValue Cmt3::getPortNr(uint8_t& port) const
{
	CMT3LOG("L3: getPortNr\n");
	CMT3EXITLOG;

	return m_lastResult = m_serial.getPortNr(port);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the product code of a device. The buffer should be at least 21 bytes.
XsensResultValue Cmt3::getProductCode (char* productCode, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getProductCode %p %08x\n",productCode,deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQPRODUCTCODE);

	uint16_t len = rcv.getDataSize();
	memcpy(productCode,rcv.getDataBuffer(),len);
	productCode[len] = '\0';

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the sample frequency of the devices on the bus.
uint16_t Cmt3::getSampleFrequency(void)
{
	CMT3LOG("L3: getSampleFrequency\n");
	CmtDeviceMode2 mode;
	mode.m_period = m_period;
	mode.m_skip = m_skip;

	return mode.getSampleFrequency();
}

//////////////////////////////////////////////////////////////////////////////////////////
// Get the baudrate that is reported for the serial connection
XsensResultValue Cmt3::getSerialBaudrate(uint32_t& baudrate)
{
	CMT3LOG("L3: getSerialBaudrate\n");
	CMT3EXITLOG;

	DO_DATA_REQUEST_BID(CMT_MID_REQBAUDRATE,CMT_BID_MASTER);
	switch (rcv.getDataByte())
	{
	case CMT_BAUDCODE_9K6:
		baudrate = CMT_BAUD_RATE_9600;
		break;
//	case CMT_BAUDCODE_14K4:
//		baudrate = CMT_BAUD_RATE_14K4;
//		break;
	case CMT_BAUDCODE_19K2:
		baudrate = CMT_BAUD_RATE_19K2;
		break;
//	case CMT_BAUDCODE_28K8:
//		baudrate = CMT_BAUD_RATE_28K8;
//		break;
	case CMT_BAUDCODE_38K4:
		baudrate = CMT_BAUD_RATE_38K4;
		break;
	case CMT_BAUDCODE_57K6:
		baudrate = CMT_BAUD_RATE_57K6;
		break;
//	case CMT_BAUDCODE_76K8:
//		baudrate = CMT_BAUD_RATE_76K8;
//		break;
	case CMT_BAUDCODE_115K2:
		baudrate = CMT_BAUD_RATE_115K2;
		break;
	case CMT_BAUDCODE_230K4:
		baudrate = CMT_BAUD_RATE_230K4;
		break;
	case CMT_BAUDCODE_460K8:
		baudrate = CMT_BAUD_RATE_460K8;
		break;
	case CMT_BAUDCODE_921K6:
		baudrate = CMT_BAUD_RATE_921K6;
		break;
	default:
		return m_lastResult = XRV_BAUDRATEINVALID;
	}
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the inbound synchronization settings of a device.
XsensResultValue Cmt3::getSyncInSettings(CmtSyncInSettings& settings)
{
	CMT3LOG("L3: getSyncInSettings\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCINSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCIN_MODE);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_mode = rcv.getDataShort(1);

	snd.setDataByte(CMT_PARAM_SYNCIN_SKIPFACTOR);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_skipFactor = rcv.getDataShort(1);

	snd.setDataByte(CMT_PARAM_SYNCIN_OFFSET);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_offset = rcv.getDataLong(1);
	
	// convert the offset to ns
	settings.m_offset = (uint32_t) ((((double)settings.m_offset)*CMT_SYNC_CLOCK_TICKS_TO_NS)+0.5);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the inbound synchronization mode of a device.
XsensResultValue Cmt3::getSyncInMode(uint16_t& mode)
{
	CMT3LOG("L3: getSyncInMode\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCINSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCIN_MODE);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	mode = rcv.getDataShort(1);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the inbound synchronization skip factor of a device.
XsensResultValue Cmt3::getSyncInSkipFactor(uint16_t& skipFactor)
{
	CMT3LOG("L3: getSyncInSkipFactor\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCINSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCIN_SKIPFACTOR);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	skipFactor = rcv.getDataShort(1);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the inbound synchronization offset of a device.
XsensResultValue Cmt3::getSyncInOffset(uint32_t& offset)
{
	CMT3LOG("L3: getSyncInOffset\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCINSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCIN_OFFSET);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	offset = rcv.getDataLong(1);

	// convert the offset to ns
	offset = (uint32_t) ((((double)offset)*CMT_SYNC_CLOCK_TICKS_TO_NS)+0.5);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Get the synchronization mode of the XM
XsensResultValue Cmt3::getSyncMode(uint8_t& mode)
{
	CMT3LOG("L3: getSyncMode\n");
	CMT3EXITLOG;

	DO_DATA_REQUEST_BID(CMT_MID_REQSYNCMODE,CMT_BID_MASTER);
	mode = rcv.getDataByte();
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the outbound synchronization settings of a device.
XsensResultValue Cmt3::getSyncOutSettings(CmtSyncOutSettings& settings)
{
	CMT3LOG("L3: getSyncOutSettings\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_MODE);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_mode = rcv.getDataShort(1);

	snd.setDataByte(CMT_PARAM_SYNCOUT_SKIPFACTOR);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_skipFactor = rcv.getDataShort(1);

	snd.setDataByte(CMT_PARAM_SYNCOUT_OFFSET);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_offset = rcv.getDataLong(1);
	
	snd.setDataByte(CMT_PARAM_SYNCOUT_PULSEWIDTH);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	settings.m_pulseWidth = rcv.getDataLong(1);

	// convert the offset and pulse width to ns
	settings.m_offset = (uint32_t)	((((double)settings.m_offset)*CMT_SYNC_CLOCK_TICKS_TO_NS)+0.5);
	settings.m_pulseWidth = (uint32_t)	((((double)settings.m_pulseWidth)*CMT_SYNC_CLOCK_TICKS_TO_NS)+0.5);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the outbound synchronization mode of a device.
XsensResultValue Cmt3::getSyncOutMode(uint16_t& mode)
{
	CMT3LOG("L3: getSyncOutMode\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_MODE);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	mode = rcv.getDataShort(1);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the outbound synchronization pulse width of a device.
XsensResultValue Cmt3::getSyncOutPulseWidth(uint32_t& pulseWidth)
{
	CMT3LOG("L3: getSyncOutPulseWidth\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_PULSEWIDTH);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	pulseWidth = rcv.getDataLong(1);

	// convert the pulse width to ns
	pulseWidth = (uint32_t)	((((double)pulseWidth)*CMT_SYNC_CLOCK_TICKS_TO_NS)+0.5);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the outbound synchronization skip factor of a device.
XsensResultValue Cmt3::getSyncOutSkipFactor(uint16_t& skipFactor)
{
	CMT3LOG("L3: getSyncOutSkipFactor\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_SKIPFACTOR);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	skipFactor = rcv.getDataShort(1);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the outbound synchronization offset of a device.
XsensResultValue Cmt3::getSyncOutOffset(uint32_t& offset)
{
	CMT3LOG("L3: getSyncOutOffset\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,1);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_OFFSET);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	offset = rcv.getDataLong(1);

	// convert the offset to ns
	offset = (uint32_t)	((((double)offset)*CMT_SYNC_CLOCK_TICKS_TO_NS)+0.5);

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
uint32_t Cmt3::getTimeoutConfig (void) const
{
	return m_timeoutConf;
}

//////////////////////////////////////////////////////////////////////////////////////////
uint32_t Cmt3::getTimeoutMeasurement (void) const
{
	return m_timeoutMeas;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the UTC time of the last received sample
XsensResultValue Cmt3::getUtcTime(CmtUtcTime& utc, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getUtcTime %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQUTCTIME);

	utc.m_nano	= rcv.getDataLong(0);
	utc.m_year	= rcv.getDataShort(4);
	utc.m_month	= rcv.getDataByte(6);
	utc.m_day	= rcv.getDataByte(7);
	utc.m_hour	= rcv.getDataByte(8);
	utc.m_minute= rcv.getDataByte(9);
	utc.m_second= rcv.getDataByte(10);
	utc.m_valid	= rcv.getDataByte(11);

//	if (utc.m_valid == 0)
//		return m_lastResult = XRV_INVALID_TIME;

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Get the dual-mode output settings of the XM
XsensResultValue Cmt3::getXmOutputMode(uint8_t& mode)
{
	CMT3LOG("L3: getXmOutputMode\n");
	CMT3EXITLOG;

	DO_DATA_REQUEST_BID(CMT_MID_REQOPMODE,CMT_BID_MASTER);
	mode = rcv.getDataByte();
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Place all connected sensors into Configuration Mode.
XsensResultValue Cmt3::gotoConfig(void)
{
	CMT3LOG("L3: gotoConfig port %u\n",(uint32_t)m_serial.getCmt1s()->getPortNr());
	CMT3EXITLOG;

	Message snd(CMT_MID_GOTOCONFIG);
	Message rcv;
	int32_t tries = 0;

	srand( (unsigned int)timeStampNow());

	m_serial.setTimeout(CMT3_CONFIG_TIMEOUT);
	snd.setBusId(CMT_BID_MASTER);
	while (tries++ < m_gotoConfigTries)
	{
		m_serial.getCmt1s()->flushData();			// special case for goto config. we want the buffer to be empty
		CMT3LOG("L3: Attempt to goto config %d\n",tries);
		m_serial.writeMessage(&snd);
		m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_GOTOCONFIGACK,0,false);
		if (m_lastResult == XRV_TIMEOUTNODATA)
			break;
		if (m_lastResult == XRV_OK)
		{
			if (m_logging)
				m_logFile.writeMessage(&rcv);
			if (rcv.getMessageId() == CMT_MID_ERROR)
			{
				m_lastHwErrorDeviceId = CMT_DID_MASTER;
				if (rcv.getDataSize() >= 2)
				{
					uint8_t biddy = rcv.getDataByte(1);
					getDeviceId(biddy,m_lastHwErrorDeviceId);
				}
				m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte(0);
				CMT3LOG("L3: Goto config failed, error received %d: %s\n",(int32_t)m_lastResult,xsensResultText(m_lastResult));
				m_serial.setTimeout(m_timeoutConf);
				return m_lastResult;
			}
			CMT3LOG("L3: Goto config succeeded\n");
			m_measuring = false;
			m_serial.setTimeout(m_timeoutConf);
			return m_lastResult = XRV_OK;
		}
		m_lastResult = m_serial.getLastResult();
		msleep(((long)rand() * 10)/RAND_MAX);
	}
	m_serial.setTimeout(m_timeoutConf);
	m_measuring = (m_lastResult != XRV_OK);
	CMT3LOG("L3: Goto config returns %d: %s\n",(int32_t)m_lastResult,xsensResultText(m_lastResult));
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Place all connected sensors into Measurement Mode.
XsensResultValue Cmt3::gotoMeasurement(void)
{
	CMT3LOG("L3: gotoMeasurement port %u\n",(uint32_t)m_serial.getCmt1s()->getPortNr());
	CMT3EXITLOG;

	Message snd(CMT_MID_GOTOMEASUREMENT);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_GOTOMEASUREMENTACK,0,false);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	m_rtcInitialized = false;
	m_measuring = true;
	m_serial.setTimeout(m_timeoutMeas);
	return (m_lastResult = XRV_OK);
}

XsensResultValue Cmt3::initBus(void)
{
	CMT3LOG("L3: initBus port %u\n",(uint32_t)m_serial.getCmt1s()->getPortNr());
	CMT3EXITLOG;

	DO_DATA_REQUEST_BID(CMT_MID_INITBUS,CMT_BID_MASTER);
	return m_lastResult = XRV_OK;	// m_serial.waitForMessage(&rcv,CMT_MID_INITBUSRESULTS);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return whether the main device is an Xbus Master or not.
bool Cmt3::isXm(void) const
{
	return ((m_config.m_masterDeviceId & CMT_DID_TYPEH_MASK) == CMT_DID_TYPEH_XM); 
}

//////////////////////////////////////////////////////////////////////////////////////////
// Open a communication channel to the given serial port name.
XsensResultValue Cmt3::openPort(const char *portName, const uint32_t baudRate)
{
	CMT3LOG("L3: openPort Opening port %s @baud %d, timeoutC=%u, timeoutM=%u\n", portName, baudRate, m_timeoutConf, m_timeoutMeas);
	CMT3EXITLOG;
	
	if (m_logFile.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;
		
	m_serial.setTimeout(m_timeoutConf);
	if ((m_lastResult = m_serial.open(portName, baudRate)) != XRV_OK)
		return m_lastResult;
	
	CMT3LOG("L3: openPort: Low level port opened, gotoConfig\n");

    m_baudrate = baudRate;
    m_rtcInitialized = false;
    m_measuring = true;         // required for faster operation of refreshCache
    m_logging = false;

    // place the device in config mode
    if (gotoConfig() != XRV_OK)
    {
        CMT3LOG("L3: openPort: gotoConfig failed: [%d]%s\n",m_lastResult,xsensResultText(m_lastResult));
        m_serial.close();
        return XRV_CONFIGCHECKFAIL;
    }

    CMT3LOG("L3: openPort: gotoConfig succeeded, requesting initBus\n");
    Message snd,rcv;

    if (initBus() != XRV_OK)
    {
        CMT3LOG("L3: openPort: initBus failed: [%d]%s\n",m_lastResult,xsensResultText(m_lastResult));
        m_serial.close();
        return XRV_CONFIGCHECKFAIL;
    }

    CMT3LOG("L3: openPort: initBus succeeded, cleaning up the cache\n");
    if (refreshCache() != XRV_OK)
    {
        m_serial.close();
        CMT3LOG("L3: openPort: refreshCache failed: [%d]%s\n",m_lastResult,xsensResultText(m_lastResult));
        return XRV_CONFIGCHECKFAIL;
    }

    CMT3LOG("L3: openPort: returning OK\n");
    return m_lastResult = XRV_OK;
}

#ifdef _WIN32
//////////////////////////////////////////////////////////////////////////////////////////
// Open a communication channel to the given COM port number.
XsensResultValue Cmt3::openPort(const uint32_t portNumber, const uint32_t baudRate)
{
	// open the port
	CMT3LOG("L3: openPort Opening port %d @baud %d, timeoutC=%u, timeoutM=%u\n",(int32_t)portNumber,baudRate,m_timeoutConf, m_timeoutMeas);
	CMT3EXITLOG;

	if (m_logFile.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;

	m_serial.setTimeout(m_timeoutConf);	// then update L2 (and L1)
	if ((m_lastResult = m_serial.open(portNumber, baudRate)) != XRV_OK)
		return m_lastResult;

	CMT3LOG("L3: openPort: Low level port opened, gotoConfig\n");

	m_baudrate = baudRate;
	m_rtcInitialized = false;
	m_measuring = true;			// required for faster operation of refreshCache
	m_logging = false;

	// place the device in config mode
	if (gotoConfig() != XRV_OK)
	{
		CMT3LOG("L3: openPort: gotoConfig failed: [%d]%s\n",m_lastResult,xsensResultText(m_lastResult));
		m_serial.close();
		return XRV_CONFIGCHECKFAIL;
	}

	CMT3LOG("L3: openPort: gotoConfig succeeded, requesting initBus\n");
	Message snd,rcv;

	if (initBus() != XRV_OK)
	{
		CMT3LOG("L3: openPort: initBus failed: [%d]%s\n",m_lastResult,xsensResultText(m_lastResult));
		m_serial.close();
		return XRV_CONFIGCHECKFAIL;
	}

	CMT3LOG("L3: openPort: initBus succeeded, cleaning up the cache\n");
	if (refreshCache() != XRV_OK)
	{
		m_serial.close();
		CMT3LOG("L3: openPort: refreshCache failed: [%d]%s\n",m_lastResult,xsensResultText(m_lastResult));
		return XRV_CONFIGCHECKFAIL;
	}

	CMT3LOG("L3: openPort: returning OK\n");
	return m_lastResult = XRV_OK;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Get the MessageId of the next logged message
XsensResultValue Cmt3::peekLogMessageId(uint8_t& messageId)
{
	CMT3LOG("L3: peekLogMessageId started\n");
	CMT3EXITLOG;

	if (!m_readFromFile)
		return m_lastResult = XRV_INVALIDOPERATION;

	Message msg;

	CmtFilePos pos =  m_logFile.getReadPosition();
	m_lastResult = m_logFile.readMessage(&msg);
	m_logFile.setReadPosition(pos);

	if (m_lastResult != XRV_OK)
	{
		CMT3LOG("L3: peekLogMessageId, no messages to be read\n");
		return m_lastResult;
	}
	messageId = msg.getMessageId();
	CMT3LOG("L3: peekLogMessageId found msg with ID %02x\n",(int32_t) messageId);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve a data message.
XsensResultValue Cmt3::readDataPacket(Packet* pack, bool acceptOther)
{
	CMT3LOGDAT("L3: readDataPacket %p %u\n",pack,acceptOther?1:0);
	CMT3EXITLOGDAT;

	if (!m_readFromFile)
	{
		while(1)
		{
			m_lastResult = m_serial.readMessage(&pack->m_msg);
			if (m_lastResult != XRV_OK)
			{
				CMT3LOGDAT("L3: readDataPacket, no data messages to be read\n");
				return m_lastResult;
			}
			if (m_logging)
				m_logFile.writeMessage(&pack->m_msg);
			if (pack->m_msg.getMessageId() == CMT_MID_MTDATA)
			{
				pack->setXbus(m_config.m_masterDeviceId != m_config.m_deviceInfo[0].m_deviceId,false);
				pack->m_itemCount = m_config.m_numberOfDevices;
				for (uint16_t i = 0;i < m_config.m_numberOfDevices;++i)
					pack->setDataFormat(m_config.m_deviceInfo[i].m_outputMode,m_config.m_deviceInfo[i].m_outputSettings,i);
				pack->m_toa = timeStampNow();
				if (m_useRtc)
					fillRtc(pack);
				CMT3LOGDAT("L3: readDataPacket, data message read\n");
				return m_lastResult = XRV_OK;
			}
			else if (pack->m_msg.getMessageId() == CMT_MID_ERROR)
			{
				m_lastHwErrorDeviceId = m_config.m_masterDeviceId;
				if (pack->m_msg.getDataSize() >= 2)
				{
					uint8_t biddy = pack->m_msg.getDataByte(1);
					getDeviceId(biddy,m_lastHwErrorDeviceId);
				}
				return m_lastResult = m_lastHwError = (XsensResultValue) pack->m_msg.getDataByte(0);
			}
			CMT3LOGDAT("L3: readDataPacket, non-data message read: %2x\n",(int32_t) pack->m_msg.getMessageId());
			if (acceptOther)
			{
				CMT3LOGDAT("L3: accepting other message\n");
				return m_lastResult = XRV_OTHER;
			}
		}
	}
	else
	{
		while(1)
		{
			m_lastResult = m_logFile.readMessage(&pack->m_msg);
			if (m_lastResult != XRV_OK)
			{
				CMT3LOGDAT("L3: readDataPacket, no data messages to be read\n");
				return m_lastResult;
			}
			if (pack->m_msg.getMessageId() == CMT_MID_MTDATA)
			{
				pack->setXbus(m_config.m_masterDeviceId != m_config.m_deviceInfo[0].m_deviceId,false);
				pack->m_itemCount = m_config.m_numberOfDevices;
				for (uint16_t i = 0;i < m_config.m_numberOfDevices;++i)
					pack->setDataFormat(m_config.m_deviceInfo[i].m_outputMode,m_config.m_deviceInfo[i].m_outputSettings,i);
				pack->m_toa = timeStampNow();
				if (m_useRtc)
					fillRtc(pack);
				CMT3LOGDAT("L3: readDataPacket, data message read\n");
				return m_lastResult = XRV_OK;
			}
			else if (pack->m_msg.getMessageId() == CMT_MID_ERROR)
			{
				m_lastHwErrorDeviceId = m_config.m_masterDeviceId;
				if (pack->m_msg.getDataSize() >= 2)
				{
					uint8_t biddy = pack->m_msg.getDataByte(1);
					getDeviceId(biddy,m_lastHwErrorDeviceId);
				}
				return m_lastResult = m_lastHwError = (XsensResultValue) pack->m_msg.getDataByte(0);
			}
			CMT3LOGDAT("L3: readDataPacket, non-data message read: %2x\n",(int32_t) pack->m_msg.getMessageId());
			if (acceptOther)
			{
				CMT3LOGDAT("L3: accepting other message\n");
				return m_lastResult = XRV_OTHER;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////
// Request a data message and wait for it to arrive.
XsensResultValue Cmt3::requestData(Packet* pack)
{
	CMT3LOGDAT("L3: requestData %p\n",pack);
	CMT3EXITLOGDAT;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;
	Message snd(CMT_MID_REQDATA);
	m_serial.writeMessage(&snd);
	return readDataPacket(pack);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Reset all connected sensors.
XsensResultValue Cmt3::reset(void)
{
	CMT3LOG("L3: reset\n");
	CMT3EXITLOG;

	Message snd(CMT_MID_RESET,0);
	Message rcv;
	snd.setBusId(CMT_BID_MASTER);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_RESETACK,0,false);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	m_measuring = true;
	refreshCache();
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
//! Perform an orientation reset on a device.
XsensResultValue Cmt3::resetOrientation(const CmtResetMethod method, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: resetOrientation %u %08x\n",(uint32_t) method,deviceId);
	CMT3EXITLOG;

	DO_DATA_SET(CMT_MID_RESETORIENTATION,CMT_LEN_RESETORIENTATION,Short,(uint16_t) method);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
//! Restore the factory defaults of a device.
XsensResultValue Cmt3::restoreFactoryDefaults(const CmtDeviceId deviceId)
{
	CMT3LOG("L3: restoreFactoryDefaults %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_RESTOREFACTORYDEF);
	return m_lastResult = XRV_OK;
}

#if 0
obsolete:
//////////////////////////////////////////////////////////////////////////////////////////
// Set the state (enabled/disabled) of the AMD algorithm
XsensResultValue Cmt3::setAmdState (const bool state, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setAmdState %u %08x\n",state?1:0,deviceId);
	CMT3EXITLOG;

	uint16_t dat = ((state)?1:0);
	DO_DATA_SET(CMT_MID_REQAMD,CMT_LEN_AMD,Short,dat);
	return m_lastResult = XRV_OK;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Set the baudrate and reconnect at the new baudrate if successful.
XsensResultValue Cmt3::setBaudrate(const uint32_t baudrate, bool reconnect)
{
	CMT3LOG("L3: setBaudrate %u %u\n",baudrate,reconnect?1:0);
	CMT3EXITLOG;

	uint8_t tmp;
	switch (baudrate)
	{
	case CMT_BAUD_RATE_9600:
		tmp = CMT_BAUDCODE_9K6;
		break;
//	case CMT_BAUD_RATE_14K4:
//		tmp = CMT_BAUDCODE_14K4;
//		break;
	case CMT_BAUD_RATE_19K2:
		tmp = CMT_BAUDCODE_19K2;
		break;
//	case CMT_BAUD_RATE_28K8:
//		tmp = CMT_BAUDCODE_28K8;
//		break;
	case CMT_BAUD_RATE_38K4:
		tmp = CMT_BAUDCODE_38K4;
		break;
	case CMT_BAUD_RATE_57K6:
		tmp = CMT_BAUDCODE_57K6;
		break;
//	case CMT_BAUD_RATE_76K8:
//		tmp = CMT_BAUDCODE_76K8;
//		break;
	case CMT_BAUD_RATE_115K2:
		tmp = CMT_BAUDCODE_115K2;
		break;
	case CMT_BAUD_RATE_230K4:
		tmp = CMT_BAUDCODE_230K4;
		break;
	case CMT_BAUD_RATE_460K8:
		tmp = CMT_BAUDCODE_460K8;
		break;
	case CMT_BAUD_RATE_921K6:
		tmp = CMT_BAUDCODE_921K6;
		break;
	default:
		return m_lastResult = XRV_BAUDRATEINVALID;
	}

	if (baudrate != m_baudrate) {
		DO_DATA_SET_BID(CMT_MID_REQBAUDRATE,CMT_LEN_BAUDRATE,Byte,tmp,CMT_BID_MASTER);

		if (reconnect)
		{
			CMT3LOG("L3: sending reset for reconnect\n");
			// Reset devices on this port and reopen port @ new baudrate
			Message sndReset(CMT_MID_RESET,0);
			Message rcvReset;
			sndReset.setBusId(CMT_BID_MASTER);
			m_serial.writeMessage(&sndReset);
			m_lastResult = m_serial.waitForMessage(&rcvReset,CMT_MID_RESETACK,0,false);
			if (m_lastResult != XRV_OK)
				return m_lastResult;
			if (m_logging)
				m_logFile.writeMessage(&rcvReset);
			bool wasMeasuring = m_measuring;
			m_measuring = true;

			CMT3LOG("L3: reopening port at new baud rate\n");
#ifdef _WIN32
			int32_t port;
			m_serial.getPortNr(port);
			closePort(false);
			m_lastResult = openPort(port,baudrate);
#else
			char portname[32];
			m_serial.getPortName(portname);
			closePort(false);
			m_lastResult = openPort(portname, baudrate);
#endif
			if (m_lastResult != XRV_OK)
				return m_lastResult;
			if (wasMeasuring)
				gotoMeasurement();
			return m_lastResult;
		}
	}

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the state of the bluetooth communication to on (true) or off (false)
XsensResultValue Cmt3::setBluetoothState(const bool enabled)
{
	CMT3LOG("L3: setBluetoothState %u\n",enabled?1:0);
	CMT3EXITLOG;

	uint8_t dat = (enabled?0:1);
	DO_DATA_SET_BID(CMT_MID_REQBTDISABLE,CMT_LEN_BTDISABLE,Byte,dat,CMT_BID_MASTER);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Switch the XM bus power on or off.
XsensResultValue Cmt3::setBusPowerState(const bool enabled)
{
	CMT3LOG("L3: setBusPowerState %u\n",enabled?1:0);
	CMT3EXITLOG;

	uint16_t tmp = enabled?1:0;
	DO_DATA_SET_BID(CMT_MID_BUSPWR,CMT_LEN_BUSPWR,Short,tmp,CMT_BID_MASTER);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the complete device output mode of a device.
XsensResultValue Cmt3::setDeviceMode (const CmtDeviceMode& mode, bool force, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setDeviceMode %04x %08x %u %u %08x\n",(uint32_t) mode.m_outputMode,(uint32_t) mode.m_outputSettings,(uint32_t) mode.m_sampleFrequency,force?1:0,deviceId);
	CMT3EXITLOG;

	CmtDeviceMode2 mode2;
	mode2.m_outputMode = mode.m_outputMode;
	mode2.m_outputSettings = mode.m_outputSettings;
	mode2.setSampleFrequency(mode.m_sampleFrequency);

	return setDeviceMode2(mode2, force, deviceId);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the complete device output mode of a device.
XsensResultValue Cmt3::setDeviceMode2 (const CmtDeviceMode2& mode, bool force, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setDeviceMode2 %04x %08x %u %u %u %08x\n",(uint32_t) mode.m_outputMode,(uint32_t) mode.m_outputSettings,(uint32_t) mode.m_period, (uint32_t)mode.m_skip,force?1:0,deviceId);
	CMT3EXITLOG;

	Message snd;
	Message rcv;
	uint8_t bid = getBusIdInternal(deviceId);
	if (bid == CMT_BID_INVALID)
		return (m_lastResult = XRV_INVALIDID);
	
	//uint16_t period, skip;
	//mode.getPeriodAndSkipFactor(period, skip);
	uint16_t xperiod;
	//bool changed = false;

	bool xm = isXm();
	if (bid == CMT_BID_BROADCAST)
	{
		if (xm)
		{
			// set the device modes of all connected devices first
			for (uint16_t i = 0; i < m_config.m_numberOfDevices;++i)
				if ((m_lastResult = setDeviceMode2(mode,force,m_config.m_deviceInfo[i].m_deviceId)) != XRV_OK)
					return m_lastResult;
		}
		bid = CMT_BID_MASTER;
	}

	// set sample frequency if this is an Xbus Master or the primary device
	if (deviceId == CMT_DID_BROADCAST || deviceId == m_config.m_masterDeviceId)
	{
		m_period = mode.m_period;
		m_skip = mode.m_skip;
		//m_sampleFrequency = (double) mode.getSampleFrequency();
		snd.setBusId(CMT_BID_MASTER);
		if (xm)
			xperiod = mode.m_period * (mode.m_skip + 1);
		else
			xperiod = mode.m_period;

		if (force || m_config.m_samplingPeriod != xperiod)
		{
			CMT3LOG("L3: setDeviceMode setting device %08x period to %u\n",deviceId,(uint32_t) xperiod);
			//changed = true;
			snd.setDataShort(xperiod);
			snd.setMessageId(CMT_MID_REQPERIOD);
			m_serial.writeMessage(&snd);
			m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQPERIODACK,0,true);
			if (m_lastResult != XRV_OK)
				return m_lastResult;
			if (m_logging)
				m_logFile.writeMessage(&rcv);
			HANDLE_ERR_RESULT;
			m_config.m_samplingPeriod = xperiod;	// update device info
		}

		if (!xm && (force || m_config.m_outputSkipFactor != mode.m_skip))
		{
			CMT3LOG("L3: setDeviceMode setting MT %08x skip factor to %u\n",deviceId,(uint32_t) mode.m_skip);
			//changed = true;
			snd.setDataShort(mode.m_skip);
			snd.setMessageId(CMT_MID_REQOUTPUTSKIPFACTOR);
			m_serial.writeMessage(&snd);
			m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQOUTPUTSKIPFACTORACK,0,true);
			if (m_lastResult != XRV_OK)
				return m_lastResult;
			if (m_logging)
				m_logFile.writeMessage(&rcv);
			HANDLE_ERR_RESULT;
			m_config.m_outputSkipFactor = mode.m_skip;	// update device info
		}
	}

	// set the output mode and settings if this device is not an Xbus Master
	if ((deviceId & CMT_DID_TYPEH_MASK) != CMT_DID_TYPEH_XM)
	{
		if (bid == CMT_BID_BROADCAST || bid == CMT_BID_MASTER) {
			snd.setBusId(CMT_BID_MASTER);
			bid = 1;
		}
		else
			snd.setBusId(bid);
	
		if (force || m_config.m_deviceInfo[bid-1].m_outputMode != (uint16_t) mode.m_outputMode)
		{
			CMT3LOG("L3: setDeviceMode setting MT %08x output mode to %04X\n",deviceId,(uint32_t) mode.m_outputMode);
			//changed = true;
			snd.resizeData(2);
			snd.setMessageId(CMT_MID_REQOUTPUTMODE);
			snd.setDataShort((uint16_t) mode.m_outputMode);
			m_serial.writeMessage(&snd);
			m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQOUTPUTMODEACK,0,true);
			if (m_lastResult != XRV_OK)
				return m_lastResult;
			if (m_logging)
				m_logFile.writeMessage(&rcv);
			HANDLE_ERR_RESULT;
			m_config.m_deviceInfo[bid-1].m_outputMode = (uint16_t) mode.m_outputMode;	// update device info
		}

		uint32_t settings = (uint32_t) mode.m_outputSettings;
		if (xm)
			settings &= ~(uint32_t) CMT_OUTPUTSETTINGS_TIMESTAMP_MASK;

		if (force || m_config.m_deviceInfo[bid-1].m_outputSettings != settings)
		{
			CMT3LOG("L3: setDeviceMode setting MT %08x output settings to %08X\n",deviceId,(uint32_t) settings);
			//changed = true;
			snd.setMessageId(CMT_MID_REQOUTPUTSETTINGS);
			snd.setDataLong(settings);
			m_serial.writeMessage(&snd);
			m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQOUTPUTSETTINGSACK,0,true);
			if (m_lastResult != XRV_OK)
				return m_lastResult;
			if (m_logging)
				m_logFile.writeMessage(&rcv);
			HANDLE_ERR_RESULT;
			m_config.m_deviceInfo[bid-1].m_outputSettings = settings;	// update device info
		}
	}

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the error mode
XsensResultValue Cmt3::setErrorMode(const uint16_t mode)
{
	CMT3LOG("L3: setErrorMode %u\n",(uint32_t) mode);
	CMT3EXITLOG;

	uint8_t mid;
	if (isXm())
		mid = CMT_MID_REQXMERRORMODE;
	else
		mid = CMT_MID_REQERRORMODE;

	DO_DATA_SET_BID(mid,CMT_LEN_ERRORMODE,Short,mode,CMT_BID_MASTER);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the number of times the gotoConfig function will attempt gotoConfig before failing
XsensResultValue Cmt3::setGotoConfigTries(const uint16_t tries)
{
	CMT3LOG("L3: setGotoConfigTries %u\n",(uint32_t) tries);
	CMT3EXITLOG;

	m_gotoConfigTries = tries;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the heading offset of a device. The valid range is -pi to +pi.
XsensResultValue Cmt3::setHeading (const double heading, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setHeading %f %08x\n",heading,deviceId);
	CMT3EXITLOG;

	DO_DATA_SET(CMT_MID_REQHEADING,CMT_LEN_HEADING,Float,(float) heading);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the location ID of a device. The buffer should be no more than 20 bytes.
XsensResultValue Cmt3::setLocationId (uint16_t locationId, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setLocationId %u %08x\n",(uint32_t) locationId,deviceId);
	CMT3EXITLOG;

	DO_DATA_SET(CMT_MID_REQLOCATIONID,CMT_LEN_LOCATIONID,Short,locationId);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the magnetic declination offset of a device. The valid range is -pi to +pi.
XsensResultValue Cmt3::setMagneticDeclination(const double declination, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setMagneticDeclination %f %08x\n",declination,deviceId);
	CMT3EXITLOG;

	DO_DATA_SET(CMT_MID_SETMAGNETICDECLINATION,CMT_LEN_MAGNETICDECLINATION,Float,(float) declination);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Switch the XM off
XsensResultValue Cmt3::setXmPowerOff(void)
{
	CMT3LOG("L3: setXmPowerOff\n");
	CMT3EXITLOG;

	if (!isXm())
		return m_lastResult = XRV_INVALIDOPERATION;
	Message snd(CMT_MID_XMPWROFF,0);
	snd.setBusId(CMT_BID_MASTER);
	return m_lastResult = m_serial.writeMessage(&snd);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the inbound synchronization settings of a device.
XsensResultValue Cmt3::setSyncInSettings (const CmtSyncInSettings& settings)
{
	CMT3LOG("L3: setSyncInSettings %u %u %u\n",(uint32_t) settings.m_mode,settings.m_offset,(uint32_t) settings.m_skipFactor);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCINSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCIN_MODE);
	snd.setDataShort(settings.m_mode,1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	snd.setDataByte(CMT_PARAM_SYNCIN_SKIPFACTOR);
	snd.setDataShort(settings.m_skipFactor,1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	snd.setDataByte(CMT_PARAM_SYNCIN_OFFSET);
	snd.setDataLong((uint32_t) (((double) settings.m_offset)
		* CMT_SYNC_CLOCK_NS_TO_TICKS + 0.5),1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult == XRV_OK)
	{
		if (m_logging)
			m_logFile.writeMessage(&rcv);
		HANDLE_ERR_RESULT;
	}

	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the inbound synchronization mode of a device.
XsensResultValue Cmt3::setSyncInMode (const uint16_t mode)
{
	CMT3LOG("L3: setSyncInMode %u\n",(uint32_t) mode);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCINSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCIN_MODE);
	snd.setDataShort(mode,1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the inbound synchronization skip factor of a device.
XsensResultValue Cmt3::setSyncInSkipFactor (const uint16_t skipFactor)
{
	CMT3LOG("L3: setSyncInSettings %u\n",skipFactor);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCINSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCIN_SKIPFACTOR);
	snd.setDataShort(skipFactor,1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the inbound synchronization offset of a device.
XsensResultValue Cmt3::setSyncInOffset (const uint32_t offset)
{
	CMT3LOG("L3: setSyncInSettings %u\n",offset);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCINSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCIN_OFFSET);
	snd.setDataLong((uint32_t) (((double) offset)
		* CMT_SYNC_CLOCK_NS_TO_TICKS + 0.5),1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCINSETTINGSACK,0,true);
	if (m_lastResult == XRV_OK)
	{
		if (m_logging)
			m_logFile.writeMessage(&rcv);
		HANDLE_ERR_RESULT;
	}

	return m_lastResult;
}


//////////////////////////////////////////////////////////////////////////////////////////
// Set the synchronization mode of the XM
XsensResultValue Cmt3::setSyncMode(const uint8_t mode)
{
	CMT3LOG("L3: setSyncMode %u\n",(uint32_t) mode);
	CMT3EXITLOG;

	if (!isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	DO_DATA_SET_BID(CMT_MID_REQSYNCMODE,CMT_LEN_SYNCMODE,Byte,mode,CMT_BID_MASTER);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the outbound synchronization settings of a device.
XsensResultValue Cmt3::setSyncOutSettings (const CmtSyncOutSettings& settings)
{
	CMT3LOG("L3: setSyncOutSettings %u %u %u %u\n",(uint32_t) settings.m_mode, settings.m_offset, (uint32_t) settings.m_pulseWidth, (uint32_t) settings.m_skipFactor);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_MODE);
	snd.setDataShort(settings.m_mode,1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	snd.setDataByte(CMT_PARAM_SYNCOUT_SKIPFACTOR);
	snd.setDataShort(settings.m_skipFactor,1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	snd.setDataByte(CMT_PARAM_SYNCOUT_OFFSET);
	snd.setDataLong((uint32_t) (((double) settings.m_offset)
								* CMT_SYNC_CLOCK_NS_TO_TICKS + 0.5),1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;
	
	snd.setDataByte(CMT_PARAM_SYNCOUT_PULSEWIDTH);
	snd.setDataLong((uint32_t) (((double) settings.m_pulseWidth)
								* CMT_SYNC_CLOCK_NS_TO_TICKS + 0.5),1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the outbound synchronization mode of a device.
XsensResultValue Cmt3::setSyncOutMode (const uint16_t mode)
{
	CMT3LOG("L3: setSyncOutSettings %u\n",(uint32_t) mode);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_MODE);
	snd.setDataShort(mode,1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the outbound synchronization pulse width of a device.
XsensResultValue Cmt3::setSyncOutPulseWidth(const uint32_t pulseWidth)
{
	CMT3LOG("L3: setSyncOutSettings %u\n",pulseWidth);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_PULSEWIDTH);
	snd.setDataLong((uint32_t) (((double) pulseWidth)
		* CMT_SYNC_CLOCK_NS_TO_TICKS + 0.5),1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the outbound synchronization skip factor of a device.
XsensResultValue Cmt3::setSyncOutSkipFactor(const uint16_t skipFactor)
{
	CMT3LOG("L3: setSyncOutSettings %u\n",skipFactor);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_SKIPFACTOR);
	snd.setDataShort(skipFactor,1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the outbound synchronization offset of a device.
XsensResultValue Cmt3::setSyncOutOffset(const uint32_t offset)
{
	CMT3LOG("L3: setSyncOutSettings %u\n", offset);
	CMT3EXITLOG;

	if (isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	Message snd(CMT_MID_REQSYNCOUTSETTINGS,3);
	Message rcv;

	snd.setBusId(CMT_BID_MASTER);

	snd.setDataByte(CMT_PARAM_SYNCOUT_OFFSET);
	snd.setDataLong((uint32_t) (((double) offset)
		* CMT_SYNC_CLOCK_NS_TO_TICKS + 0.5),1);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_REQSYNCOUTSETTINGSACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	HANDLE_ERR_RESULT;

	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the configuration mode timeout value in ms.
XsensResultValue Cmt3::setTimeoutConfig(const uint32_t timeout)
{
	CMT3LOG("L3: setTimeoutConfig %u\n",timeout);
	CMT3EXITLOG;

	m_timeoutConf = timeout;
	if (!m_measuring)
		return m_lastResult = m_serial.setTimeout(m_timeoutConf);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the measurement mode timeout value in ms.
XsensResultValue Cmt3::setTimeoutMeasurement(const uint32_t timeout)
{
	CMT3LOG("L3: setTimeoutMeasurement %u\n",timeout);
	CMT3EXITLOG;

	m_timeoutMeas = timeout;
	if (m_measuring)
		return m_lastResult = m_serial.setTimeout(m_timeoutMeas);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the dual-mode output settings of the XM
XsensResultValue Cmt3::setXmOutputMode(const uint8_t mode)
{
	CMT3LOG("L3: setXmOutputMode %u\n",(uint32_t) mode);
	CMT3EXITLOG;

	if (!isXm())
		return m_lastResult = XRV_INVALIDOPERATION;

	DO_DATA_SET_BID(CMT_MID_REQOPMODE,CMT_LEN_OPMODE,Byte,mode,CMT_BID_MASTER);
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::refreshCache(const bool file)
{
	CMT3LOG("L3: refreshCache %d\n",file?1:0);
	CMT3EXITLOG;

	if (m_serial.isOpen() && (!file || !m_logFile.isOpen()))
	{
		// clear eMts cache
		for (uint32_t i = 0; i < CMT_MAX_DEVICES_PER_PORT; ++i)
		{
			CHKFREENUL(m_eMtsData[i]);
		}

		// port open, go to configuration mode
		if (m_measuring && gotoConfig() != XRV_OK)
			return m_lastResult;		// m_lastResult is already set by gotoConfig()

		// now in configuration mode, read device information
		CMT3LOG("L3: refreshCache Device in configuration mode, reading device information\n");
		
		Message snd;
		Message rcv;
		
		// all information is in the Configuration message
		snd.setMessageId(CMT_MID_REQCONFIGURATION);
		m_serial.writeMessage(&snd);
		if ((m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_CONFIGURATION,0,false)) != XRV_OK)
			return m_lastResult;
		if (m_logging)
			m_logFile.writeMessage(&rcv);
		m_config.readFromMessage(rcv.getMessageStart());

		m_period = m_config.m_samplingPeriod;
		m_skip = m_config.m_outputSkipFactor;
		//CmtDeviceMode2 mode;
		//mode.setPeriodAndSkipFactor(m_config.m_samplingPeriod,m_config.m_outputSkipFactor);
		//mode.m_period = m_config.m_samplingPeriod;
		//mode.m_skip = m_config.m_outputSkipFactor;
		//m_sampleFrequency = mode.getRealSampleFrequency();

		return m_lastResult = XRV_OK;
	}
	else if (m_logFile.isOpen() && (file || !m_serial.isOpen()))
	{
		// clear eMts cache
		for (uint32_t i = 0; i < CMT_MAX_DEVICES_PER_PORT; ++i)
		{
			CHKFREENUL(m_eMtsData[i]);
		}

		// now in configuration mode, read device information
		CMT3LOG("L3: refreshCache Reading device configuration information from file\n");
		
		Message rcv;
		
		if ((m_lastResult = m_logFile.readMessage(&rcv,CMT_MID_CONFIGURATION)) != XRV_OK)
			return m_lastResult;
		m_config.readFromMessage(rcv.getMessageStart());

		m_period = m_config.m_samplingPeriod;
		m_skip = m_config.m_outputSkipFactor;
		//CmtDeviceMode2 mode;
		//mode.setPeriodAndSkipFactor(m_config.m_samplingPeriod,m_config.m_outputSkipFactor);
		//mode.m_period = m_config.m_samplingPeriod;
		//mode.m_skip = m_config.m_outputSkipFactor;
		//m_sampleFrequency = mode.getRealSampleFrequency();

		return m_lastResult = XRV_OK;
	}
	else
		return m_lastResult = XRV_NOFILEORPORTOPEN;
}

//////////////////////////////////////////////////////////////////////////////////////////
//	Wait for a data message to arrive.
XsensResultValue Cmt3::waitForDataMessage(Packet* pack)
{
	CMT3LOGDAT("L3: waitForDataMessage %p\n",pack);
	CMT3EXITLOGDAT;

	m_lastResult = XRV_TIMEOUTNODATA;

	uint32_t toEnd = (getTimeOfDay() + (uint32_t) m_timeoutMeas) % (XSENS_MS_PER_DAY);
	while(toEnd >= getTimeOfDay())
	{
		m_lastResult = m_serial.waitForMessage(&pack->m_msg,CMT_MID_MTDATA,0,true);
		if (m_lastResult == XRV_OK)
		{
			if (m_logging)
				m_logFile.writeMessage(&pack->m_msg);
			if (pack->m_msg.getMessageId() == CMT_MID_ERROR)
			{
				m_lastHwErrorDeviceId = m_config.m_masterDeviceId;
				if (pack->m_msg.getDataSize() >= 2)
				{
					uint8_t biddy = pack->m_msg.getDataByte(1);
					getDeviceId(biddy,m_lastHwErrorDeviceId);
				}
				return m_lastResult = m_lastHwError = (XsensResultValue) pack->m_msg.getDataByte(0);
			}

			pack->m_itemCount = m_config.m_numberOfDevices;
			for (uint16_t i = 0;i < m_config.m_numberOfDevices;++i)
				pack->setDataFormat(m_config.m_deviceInfo[i].m_outputMode,m_config.m_deviceInfo[i].m_outputSettings,i);
			pack->m_toa = timeStampNow();
			if (m_useRtc)
				fillRtc(pack);

			return m_lastResult = XRV_OK;
		}
	}
	return m_lastResult;// = XRV_TIMEOUT;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::createLogFile(const char* filename, bool startLogging)
{
	CMT3LOG("L3: createLogFile \"%s\" %u\n",filename?filename:"",startLogging?1:0);
	CMT3EXITLOG;

	if (!m_serial.isOpen())
		return m_lastResult = XRV_NOPORTOPEN;
	if (m_logFile.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;
	m_lastResult = m_logFile.create(filename);
	if (m_lastResult == XRV_OK)
	{
		m_logging = true;
		CmtDeviceConfiguration config; 
		if (getConfiguration(config) == XRV_OK)
		{
			void* buffer = malloc(CMT_EMTS_SIZE*(m_config.m_numberOfDevices+1));
			getEMtsData(buffer,CMT_DID_BROADCAST);
			free(buffer);
			m_logging = startLogging;
		}
	}

	if (m_lastResult != XRV_OK)
	{
		m_logFile.closeAndDelete();
		m_logging = false;
	}
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::createLogFile(const wchar_t* filename, bool startLogging)
{
	CMT3LOG("L3: createLogFile \"%S\" %u\n",filename?filename:L"",startLogging?1:0);
	CMT3EXITLOG;

	if (!m_serial.isOpen())
		return m_lastResult = XRV_NOPORTOPEN;
	if (m_logFile.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;
	m_lastResult = m_logFile.create(filename);
	if (m_lastResult == XRV_OK)
	{
		m_logging = true;
		CmtDeviceConfiguration config; 
		if (getConfiguration(config) == XRV_OK)
		{
			void* buffer = malloc(CMT_EMTS_SIZE*(m_config.m_numberOfDevices+1));
			getEMtsData(buffer,CMT_DID_BROADCAST);
			free(buffer);
			m_logging = startLogging;
		}
	}

	if (m_lastResult != XRV_OK)
	{
		m_logFile.closeAndDelete();
		m_logging = false;
	}
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::closeLogFile(bool del)
{
	CMT3LOG("L3: closeLogFile %u\n",del?1:0);
	CMT3EXITLOG;

	m_logging = false;
	if (!m_logFile.isOpen())
		return m_lastResult = XRV_NOFILEOPEN;
	if (del)
		return m_lastResult = m_logFile.closeAndDelete();
	else
		return m_lastResult = m_logFile.close();
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Cmt3::isLogFileOpen(const char* filename) const
{
	CMT3LOG("L3: isLogFileOpen \"%s\"\n",filename?filename:"");

	if (m_logFile.isOpen())
	{
		if (filename != NULL && filename[0] != 0)
		{
			char fn[CMT_MAX_FILENAME_LENGTH];
			m_logFile.getName(fn);
			if (_strnicmp(filename,fn,CMT_MAX_FILENAME_LENGTH) != 0)
				return false;
		}
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Cmt3::isLogFileOpen(const wchar_t* filename) const
{
	CMT3LOG("L3: isLogFileOpen \"%S\"\n",filename?filename:L"");

	if (m_logFile.isOpen())
	{
		if (filename != NULL && filename[0] != L'\0')
		{
			wchar_t fn[CMT_MAX_FILENAME_LENGTH];
			m_logFile.getName(fn);
			if (_wcsnicmp(filename,fn,CMT_MAX_FILENAME_LENGTH) != 0)
				return false;
		}
		return true;
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::openLogFile(const char* filename)
{
	CMT3LOG("L3: openLogFile \"%s\"\n",filename?filename:"");
	CMT3EXITLOG;

	m_logging = false;
	if (m_serial.isOpen())
		return m_lastResult = XRV_INVALIDOPERATION;
	if (m_logFile.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;
	m_lastResult = m_logFile.open(filename,true);
	if (m_lastResult == XRV_OK)
	{
		if (refreshCache() == XRV_OK)
			m_readFromFile = true;
		else
		{
			m_logFile.close();
			m_readFromFile = false;
		}
	}
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::openLogFile(const wchar_t* filename)
{
	CMT3LOG("L3: openLogFile \"%S\"\n",filename?filename:L"");
	CMT3EXITLOG;

	m_logging = false;
	if (m_serial.isOpen())
		return m_lastResult = XRV_INVALIDOPERATION;
	if (m_logFile.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;
	m_lastResult = m_logFile.open(filename,true);
	if (m_lastResult == XRV_OK)
	{
		if (refreshCache() == XRV_OK)
			m_readFromFile = true;
		else
		{
			m_logFile.close();
			m_readFromFile = false;
		}
	}
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::setDataSource(bool readFromFile)
{
	CMT3LOG("L3: setDataSource %s\n",readFromFile?"file":"port");
	CMT3EXITLOG;

	if (readFromFile)
	{
		m_logging = false;
		if (m_logFile.isOpen())
		{
			m_readFromFile = true;
			return m_lastResult = XRV_OK;
		}
		m_readFromFile = false;
		return m_lastResult = XRV_INVALIDOPERATION;
	}
	if (m_serial.isOpen())
	{
		m_readFromFile = false;
		return m_lastResult = XRV_OK;
	}
	if (m_logFile.isOpen())
	{
		m_readFromFile = true;
		return m_lastResult = XRV_INVALIDOPERATION;
	}
	m_readFromFile = false;
	return m_lastResult = XRV_NOFILEORPORTOPEN;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::setLogMode(bool active)
{
	CMT3LOG("L3: setLogMode %u\n",active?1:0);
	CMT3EXITLOG;

	if (active && (m_readFromFile || !m_logFile.isOpen()))
		return m_lastResult = XRV_NOFILEOPEN;
	m_logging = active;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::resetLogFileReadPos(void)
{
	CMT3LOG("L3: resetLogFileReadPos\n");
	CMT3EXITLOG;

	return m_lastResult = m_logFile.setReadPosition(0);
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::writeMessageToLogFile(const Message& msg)
{
	CMT3LOG("L3: writeMessageToLogFile\n");
	CMT3EXITLOG;

	if (!m_logFile.isOpen())
		return m_lastResult = XRV_NOFILEOPEN;

	return m_lastResult = m_logFile.writeMessage(&msg);
}

//////////////////////////////////////////////////////////////////////////////////////////
XsensResultValue Cmt3::getAvailableScenarios(CmtScenario* scenarios, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getAvailableScenarios %p %08x\n",scenarios,deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQAVAILABLESCENARIOS);

	char tp = 0;
	switch (deviceId & CMT_DID_TYPEH_MASK)
	{
	case CMT_DID_TYPEH_MTIG:
		tp = '6';
		break;
	case CMT_DID_TYPEH_MTI_MTX:
		tp = '3';
		break;
	}

	for (int i = 0; i < CMT_MAX_SCENARIOS_IN_MT; ++i)
	{
		scenarios[i].m_type = rcv.getDataByte(0 + i*(1+1+CMT_LEN_SCENARIOLABEL));
		scenarios[i].m_version = rcv.getDataByte(1 + i*(1+1+CMT_LEN_SCENARIOLABEL));
		memcpy(scenarios[i].m_label,rcv.getDataBuffer(2 + i*(1+1+CMT_LEN_SCENARIOLABEL)),CMT_LEN_SCENARIOLABEL);
		scenarios[i].m_label[CMT_LEN_SCENARIOLABEL] = 0;
		scenarios[i].m_filterType = tp;
	}
	return m_lastResult = XRV_OK;
}

XsensResultValue Cmt3::getScenario(uint8_t& scenarioType, uint8_t& scenarioVersion, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getScenario %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQSCENARIO);
	scenarioType = rcv.getDataByte(1);
	scenarioVersion = rcv.getDataByte(0);
	return m_lastResult = XRV_OK;
}

XsensResultValue Cmt3::setScenario(const uint8_t scenarioType, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setScenario %u %08x\n",(uint32_t) scenarioType,deviceId);
	CMT3EXITLOG;

	uint16_t scenario = (uint16_t) scenarioType;
	DO_DATA_SET(CMT_MID_SETSCENARIO,CMT_LEN_SETSCENARIO,Short,scenario);
	return m_lastResult = XRV_OK;
}

XsensResultValue Cmt3::getGravityMagnitude(double& magnitude, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getGravityMagnitude %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQGRAVITYMAGNITUDE);
	magnitude = rcv.getDataFloat(0);
	return m_lastResult = XRV_OK;
}

XsensResultValue Cmt3::setGravityMagnitude(const double magnitude, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setGravityMagnitude %f %08x\n",magnitude,deviceId);
	CMT3EXITLOG;

	DO_DATA_SET(CMT_MID_SETGRAVITYMAGNITUDE,CMT_LEN_GRAVITYMAGNITUDE,Float,(float) magnitude);
	return m_lastResult = XRV_OK;
}

XsensResultValue Cmt3::getGpsLeverArm(CmtVector& arm, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getGpsLeverArm %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQGPSLEVERARM);
	arm.m_data[0] = rcv.getDataFloat(0);
	arm.m_data[1] = rcv.getDataFloat(4);
	arm.m_data[2] = rcv.getDataFloat(8);
	return m_lastResult = XRV_OK;
}

XsensResultValue Cmt3::getGpsStatus(CmtGpsStatus& status, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: getGpsStatus %08x\n",deviceId);
	CMT3EXITLOG;

	DO_DATA_REQUEST(CMT_MID_REQGPSSTATUS);
	for (uint16_t i = 0; i < CMT_MAX_SVINFO; ++i)
	{
		status.m_svInfo[i].m_id = rcv.getDataByte(i*5+2);
		status.m_svInfo[i].m_navigationStatus = rcv.getDataByte(i*5+3);
		status.m_svInfo[i].m_signalQuality= rcv.getDataByte(i*5+4);
		status.m_svInfo[i].m_signalStrength = rcv.getDataByte(i*5+5);
	}
	return m_lastResult = XRV_OK;
}

XsensResultValue Cmt3::setGpsLeverArm(const CmtVector& arm, const CmtDeviceId deviceId)
{
	CMT3LOG("L3: setGpsLeverArm [%f %f %f] %08x\n",arm.m_data[0],arm.m_data[1],arm.m_data[2],deviceId);
	CMT3EXITLOG;

	uint8_t bid = getBusIdInternal(deviceId);
	if (bid == CMT_BID_INVALID || bid == CMT_BID_BROADCAST)
		return (m_lastResult = XRV_INVALIDID);

	Message	snd(CMT_MID_SETGPSLEVERARM,CMT_LEN_GPSLEVERARM);
	Message rcv;
	snd.setDataFloat((float) arm.m_data[0],0);
	snd.setDataFloat((float) arm.m_data[1],4);
	snd.setDataFloat((float) arm.m_data[2],8);

	snd.setBusId(bid);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_SETGPSLEVERARMACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	if (rcv.getMessageId() == CMT_MID_ERROR)
	{
		m_lastHwErrorDeviceId = m_config.m_masterDeviceId;
		if (rcv.getDataSize() >= 2)
		{
			uint8_t biddy = rcv.getDataByte(1);
			getDeviceId(biddy,m_lastHwErrorDeviceId);
		}
		return m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte();
	}
	return m_lastResult = XRV_OK;
}

XsensResultValue Cmt3::storeXkfState(const CmtDeviceId deviceId)
{
	CMT3LOG("L3: storeXkfState %08x\n",deviceId);
	CMT3EXITLOG;

	uint8_t bid = getBusIdInternal(deviceId);
	if (bid == CMT_BID_INVALID || bid == CMT_BID_BROADCAST)
		return (m_lastResult = XRV_INVALIDID);

	Message	snd(CMT_MID_STOREXKFSTATE,CMT_LEN_STOREXKFSTATE);
	Message rcv;

	snd.setBusId(bid);
	m_serial.writeMessage(&snd);
	m_lastResult = m_serial.waitForMessage(&rcv,CMT_MID_STOREXKFSTATEACK,0,true);
	if (m_lastResult != XRV_OK)
		return m_lastResult;
	if (m_logging)
		m_logFile.writeMessage(&rcv);
	if (rcv.getMessageId() == CMT_MID_ERROR)
	{
		m_lastHwErrorDeviceId = m_config.m_masterDeviceId;
		if (rcv.getDataSize() >= 2)
		{
			uint8_t biddy = rcv.getDataByte(1);
			getDeviceId(biddy,m_lastHwErrorDeviceId);
		}
		return m_lastResult = m_lastHwError = (XsensResultValue) rcv.getDataByte();
	}
	return m_lastResult = XRV_OK;
}

} // end of xsens namespace
