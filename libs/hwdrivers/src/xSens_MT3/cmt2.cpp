/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/*! \file Cmt2.cpp

	For information about objects in this file, see the appropriate header:
	\ref Cmt2.h

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
	\par 2006-04-05, v0.0.1
	\li Job Mulder:	Created
	\par 2006-07-21, v0.1.0
	\li Job Mulder:	Updated file for release 0.1.0
*/

#include "cmt2.h"

namespace xsens {

#ifdef _LOG_CMT2
#	define CMT2LOG		CMTLOG
#else
#	define CMT2LOG(...)
#endif

//////////////////////////////////////////////////////////////////////////////////////////
int32_t findValidMessage(const uint8_t* buffer, const uint16_t bufferLength)
{
	MessageHeader* hdr = NULL;
	uint16_t pre = 0;
	uint16_t target;
	bool extended;
	uint16_t length;
	int32_t res;
	Message* msg;

	// find the preamble
	while(pre < bufferLength && (buffer[pre] != CMT_PREAMBLE))
		++pre;

	if (pre >= bufferLength)
		return -1;

	// check if the message is valid
	length = bufferLength-pre;
	if (length < CMT_LEN_MSGHEADERCS)
		return -1;

	// parse header
	
	hdr = (MessageHeader*) (buffer + pre);
	if (hdr->m_length == CMT_EXTLENCODE)
	{
		extended = true;
		if (length < CMT_LEN_MSGEXTHEADERCS)
			return -1;
	}
	else
		extended = false;

	// check the reported size
	target = extended?
				((uint16_t) hdr->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) hdr->m_datlen.m_extended.m_length.m_low)
				+ CMT_LEN_MSGEXTHEADERCS:
				(uint16_t) (hdr->m_length) + CMT_LEN_MSGHEADERCS;
	if (target <= length)
	{
		// header seems to be ok, check checksum
		// check the checksum

		msg=new Message(buffer+pre,(uint16_t) target,(uint16_t) target);
		if (msg->isChecksumOk())
		{
			delete msg;
			return (int32_t) pre;
		}
		delete msg;
	}

	// try next message
	res = findValidMessage(buffer+pre+1,length-1);
	if (res == -1)
		return -1;
	return res + pre + 1;
}

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt2s  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// Default constructor, initializes all members to their default values.
Cmt2s::Cmt2s() :
	m_onMessageReceived(NULL),
	m_onMessageSent(NULL)
{
	m_lastResult = XRV_OK;
	m_readBufferCount = 0;
	m_timeout = CMT2_DEFAULT_TIMEOUT;
	m_baudrate = CMT_DEFAULT_BAUD_RATE;
	m_toEnd = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Destructor, de-initializes, frees memory allocated for buffers, etc.
Cmt2s::~Cmt2s()
{
}


XsensResultValue Cmt2s::close(void)
{
	if (m_cmt1s.getPortNr())
	{
		CMT2LOG("L2: Closing port %d\n",(int32_t)m_cmt1s.getPortNr());
		m_toEnd = 0;
		return m_lastResult = m_cmt1s.close();
	}
	else
		return m_lastResult = XRV_NOPORTOPEN;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the port that the object is connected to.
XsensResultValue Cmt2s::getPortName(char *portname) const
{
	m_cmt1s.getPortName(portname);
	// we know there should be at least "/dev/x"
	if (strlen(portname) < 6)
		return m_lastResult = XRV_ERROR;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the port that the object is connected to.
XsensResultValue Cmt2s::getPortNr(uint8_t& port) const
{
	port = m_cmt1s.getPortNr();
	if (port == 0)
		return m_lastResult = XRV_ERROR;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the port that the object is connected to.
XsensResultValue Cmt2s::getPortNr(int32_t& port) const
{
	port = m_cmt1s.getPortNr();
	if (port == 0)
		return m_lastResult = XRV_ERROR;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Open a communication channel to the given serial port name.
XsensResultValue Cmt2s::open (const char *portName, const uint32_t baudRate)
{
	CMT2LOG("L2: Opening port %s @baud %d\n", portName, baudRate);
	m_baudrate = baudRate;
	m_lastResult = m_cmt1s.open(portName, baudRate);
	m_toEnd = 0;
	CMT2LOG("L2: Port open result %d: %s\n", (int32_t) m_lastResult, xsensResultText(m_lastResult));
	return m_lastResult;
}

#ifdef _WIN32
//////////////////////////////////////////////////////////////////////////////////////////
// Open a communication channel to the given COM port number.
XsensResultValue Cmt2s::open (const uint32_t portNumber, const uint32_t baudRate)
{
	CMT2LOG("L2: Opening port %d @baud %d\n",(int32_t)portNumber,baudRate);
	m_baudrate = baudRate;
	m_lastResult = m_cmt1s.open(portNumber,baudRate);
	m_toEnd = 0;
	CMT2LOG("L2: Port open result %d: %s\n",(int32_t)m_lastResult,xsensResultText(m_lastResult));
	return m_lastResult;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
// Read a message from the COM port.
XsensResultValue Cmt2s::readMessage(Message* rcv)
{
	MessageHeader* hdr = (MessageHeader*) m_readBuffer;
	uint16_t pre = 0;
	uint32_t length = 0;
	uint32_t target=0;
	uint16_t i;
	bool extended=false;

	CMT2LOG("L2: readMessage started, bufferCount=%u\n",m_readBufferCount);

	if (m_readBufferCount == 0)
		m_readBuffer[0] = (uint8_t) ~CMT_PREAMBLE;	// create a value that is definitely NOT a preamble

	if (m_readBufferCount < CMT_MAXMSGLEN)
		m_lastResult = m_cmt1s.readData(CMT_MAXMSGLEN-m_readBufferCount,m_readBuffer+m_readBufferCount,&length);
	m_readBufferCount += (uint16_t) length;

	while(m_readBufferCount > 0)
	{
		while(m_readBufferCount > 0)
		{
			// find preamble
			while ((pre < m_readBufferCount) && (m_readBuffer[pre] != CMT_PREAMBLE))
				++pre;

			if (pre == m_readBufferCount)
			{
				CMT2LOG("L2: readMessage no preamble found in buffer\n");
				m_readBufferCount = 0;
				return m_lastResult = XRV_TIMEOUT;
			}

			CMT2LOG("L2: readMessage preamble found at position %u\n",(uint32_t) pre);
			// shift buffer to start
			if (pre)
			{
				m_readBufferCount -= pre;
				for (i=0;i<m_readBufferCount;++i)
					m_readBuffer[i] = m_readBuffer[i+pre];
			}

			if (m_readBufferCount < CMT_LEN_MSGHEADERCS)
			{
				CMT2LOG("L2: readMessage not enough header data read\n");
				return m_lastResult = XRV_TIMEOUT;
			}

			// read header
			if (hdr->m_length == CMT_EXTLENCODE)
			{
				extended = true;
				if (m_readBufferCount < CMT_LEN_MSGEXTHEADERCS)
				{
					CMT2LOG("L2: readMessage not enough extended header data read\n");
					return m_lastResult = XRV_TIMEOUT;
				}
			}
			else
				extended = false;

			// check the reported size
			target = (extended?((uint16_t) hdr->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) hdr->m_datlen.m_extended.m_length.m_low):(uint16_t) (hdr->m_length));
			CMT2LOG("L2: readMessage bytes in buffer=%u, extended=%u, target=%u\n",(uint32_t) m_readBufferCount,(uint32_t) extended, (uint32_t) target);
			if ((uint32_t) target > (uint32_t) CMT_MAXDATALEN)
			{
				// skip current preamble
				pre = 1;
				CMT2LOG("L2: readMessage invalid message length %u\n",(uint32_t) target);
				continue;
			}
			break;	// everything seems to be ok, get out of inner while() loop
		}

		// header seems to be ok, read until end and check checksum
		if (extended)
			target += CMT_LEN_MSGEXTHEADERCS;
		else
			target += CMT_LEN_MSGHEADERCS;

		// read the entire message
		if (m_readBufferCount < target)
		{
			CMT2LOG("L2: readMessage readBufferCount %u < target %u\n",(uint32_t) m_readBufferCount, (uint32_t) target);
			return m_lastResult = XRV_TIMEOUT;
		}

		// check the checksum
		if (rcv->loadFromString(m_readBuffer,(uint16_t) target) == XRV_OK)
		{
			CMT2LOG("L2: readMessage OK\n");
			if (m_onMessageReceived != NULL)
			{
				CmtBinaryData* bytes = (CmtBinaryData*) malloc(sizeof(CmtBinaryData));
				bytes->m_size = target;
				bytes->m_portNr = m_cmt1s.getPortNr();
//				bytes->m_type = CMT_CALLBACK_ONMESSAGERECEIVED;
				memcpy(bytes->m_data,m_readBuffer,target);
#ifdef _LOG_CALLBACKS
				CMTLOG("C2: m_onMessageReceived(%d,(%d,%d),%p)\n",(int32_t) m_onMessageReceivedInstance, (int32_t) bytes->m_size, (int32_t) bytes->m_portNr, m_onMessageReceivedParam);
#endif
				m_onMessageReceived(m_onMessageReceivedInstance,CMT_CALLBACK_ONMESSAGERECEIVED,bytes,m_onMessageReceivedParam);
			}

			m_readBufferCount -= (uint16_t) target;
			if (m_readBufferCount)
				for (i=0;i<m_readBufferCount;++i)
					m_readBuffer[i] = m_readBuffer[i+target];

			return m_lastResult = XRV_OK;
		}
		CMT2LOG("L2: readMessage invalid checksum %02x %02x %02x %02x %02x\n",rcv->getMessageStart()[0]
																			,rcv->getMessageStart()[1]
																			,rcv->getMessageStart()[2]
																			,rcv->getMessageStart()[3]
																			,rcv->getMessageStart()[4]);
		// skip current preamble
		pre = 1;
	}

	CMT2LOG("L2: readMessage timed out\n");
	// a timeout occurred
	return m_lastResult = XRV_TIMEOUT;
}
	
//////////////////////////////////////////////////////////////////////////////////////////
// Set the callback function for when a message has been received or sent
XsensResultValue Cmt2s::setCallbackFunction(CmtCallbackSelector tp, int32_t instance, CmtCallbackFunction func, void* param)
{
	switch (tp)
	{
	case CMT_CALLBACK_ONMESSAGERECEIVED:
		m_onMessageReceived = func;
		m_onMessageReceivedInstance = instance;
		m_onMessageReceivedParam = param;
		return m_lastResult = XRV_OK;
	case CMT_CALLBACK_ONMESSAGESENT:
		m_onMessageSent = func;
		m_onMessageSentInstance = instance;
		m_onMessageSentParam = param;
		return m_lastResult = XRV_OK;
	default:
		break;
	}
	return m_lastResult = XRV_INVALIDPARAM;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the default timeout value to use in blocking operations.
XsensResultValue Cmt2s::setTimeout (const uint32_t ms)
{
	CMT2LOG("L2: Setting timeout to %u ms\n",ms);
	if ((m_lastResult = m_cmt1s.setTimeout(ms/2)) != XRV_OK)
		return m_lastResult;		// this can't actually happen
	m_timeout = ms;
	m_toEnd = 0;
	return (m_lastResult = XRV_OK);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Wait for a message to arrive.
XsensResultValue Cmt2s::waitForMessage(Message* rcv, const uint8_t msgId, uint32_t timeoutOverride, bool acceptErrorMessage)
{
	MessageHeader* hdr = (MessageHeader*) m_readBuffer;
	uint16_t pre = 0;
	uint32_t length = 0;
	uint32_t target;
	uint16_t i;
	bool extended;
	bool readsome = (m_readBufferCount > 0);
	uint32_t toRestore = m_toEnd;

	CMT2LOG("L2: waitForMessage x%02x, TO=%u, TOend=%u, TOO=%u\n",(uint32_t) msgId, m_timeout, m_toEnd, timeoutOverride);

	// The end-time may be misinterpreted around midnight, where it may be considered
	// expired even if this is not the case. However, this is extremely rare.
	if (m_toEnd == 0)
	{
		if (timeoutOverride != 0)
			m_toEnd = (getTimeOfDay() + (uint32_t) timeoutOverride) % (XSENS_MS_PER_DAY);
		else
			m_toEnd = (getTimeOfDay() + (uint32_t) m_timeout) % (XSENS_MS_PER_DAY);
		if (m_toEnd == 0)
			m_toEnd = 1;
	}

	if (m_readBufferCount == 0)
		m_readBuffer[0] = (uint8_t) ~CMT_PREAMBLE;	// create a value that is definitely NOT a preamble

	do {
		// find preamble
		while (m_readBuffer[pre] != CMT_PREAMBLE)
		{
			if ((++pre) >= m_readBufferCount)
			{
				m_readBufferCount = 0;
				pre = 0;
				if (m_toEnd >= getTimeOfDay())
				{
					m_lastResult = m_cmt1s.readData(CMT_LEN_MSGHEADER,m_readBuffer,&length);
					m_readBufferCount = (uint16_t) length;
					if (m_readBufferCount > 0)
						readsome = true;
				}
			}
			if (m_toEnd < getTimeOfDay())
				break;
		}
		// shift buffer to start
		if (pre)
		{
			m_readBufferCount -= pre;
			for (i=0;i<m_readBufferCount;++i)
			{
				m_readBuffer[i] = m_readBuffer[i+pre];
			}
		}

		pre = 1;	// make sure we skip the first item in the next iteration
		// read header
		while (m_readBufferCount < CMT_LEN_MSGHEADERCS && (m_toEnd >= getTimeOfDay()))
		{
			m_cmt1s.readData(CMT_LEN_MSGHEADERCS - m_readBufferCount,&m_readBuffer[m_readBufferCount],&length);
			m_readBufferCount += (uint16_t) length;
		}
		if ((m_readBufferCount < CMT_LEN_MSGHEADERCS) && (m_toEnd < getTimeOfDay()))
		{
			CMT2LOG("L2: waitForMessage timeout occurred trying to read header\n");
			break;
		}

		if (hdr->m_length == CMT_EXTLENCODE)
		{
			extended = true;
			while (m_readBufferCount < CMT_LEN_MSGEXTHEADERCS && (m_toEnd >= getTimeOfDay()))
			{
				m_cmt1s.readData(CMT_LEN_MSGEXTHEADERCS - m_readBufferCount,&m_readBuffer[m_readBufferCount],&length);
				m_readBufferCount += (uint16_t) length;
			}
			if ((m_readBufferCount < CMT_LEN_MSGEXTHEADERCS) && (m_toEnd < getTimeOfDay()))
			{
				CMT2LOG("L2: waitForMessage timeout occurred trying to read extended header\n");
				break;
			}
		}
		else
			extended = false;

		// check the reported size
		if (extended && (((uint16_t) hdr->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) hdr->m_datlen.m_extended.m_length.m_low) > (uint16_t) CMT_MAXDATALEN))
			continue;

		// header seems to be ok, read until end and check checksum
		if (extended)
			target = ((uint16_t) hdr->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) hdr->m_datlen.m_extended.m_length.m_low) + CMT_LEN_MSGEXTHEADERCS;
		else
			target = hdr->m_length + CMT_LEN_MSGHEADERCS;

		// read the entire message
		while ((m_readBufferCount < target) && (m_toEnd >= getTimeOfDay()))
		{
			m_cmt1s.readData(target - m_readBufferCount,&m_readBuffer[m_readBufferCount],&length);
			m_readBufferCount += (uint16_t) length;
		}
		if ((m_readBufferCount < target) && (m_toEnd < getTimeOfDay()))
		{
			CMT2LOG("L2: waitForMessage timeout occurred\n");
			break;
		}

		// check the checksum
		//msg=new Message(m_readBuffer,(uint16_t) target, (uint16_t) target);
		if (rcv->loadFromString(m_readBuffer,(uint16_t) target) == XRV_OK)
		{
			CMT2LOG("L2: waitForMessage received msg Id x%02x while expecting x%02x, msg size=%u\n",(uint32_t) rcv->getMessageId(),(uint32_t) msgId,target);
			if (m_onMessageReceived != NULL)
			{
				CmtBinaryData* bytes = (CmtBinaryData*) malloc(sizeof(CmtBinaryData));
				bytes->m_size = target;
				bytes->m_portNr = m_cmt1s.getPortNr();
//				bytes->m_type = CMT_CALLBACK_ONMESSAGERECEIVED;
				memcpy(bytes->m_data,m_readBuffer,target);
#ifdef _LOG_CALLBACKS
				CMTLOG("C2: m_onMessageReceived(%d,(%d,%d),%p)\n",(int32_t) m_onMessageReceivedInstance, (int32_t) bytes->m_size, (int32_t) bytes->m_portNr, m_onMessageReceivedParam);
#endif
				m_onMessageReceived(m_onMessageReceivedInstance,CMT_CALLBACK_ONMESSAGERECEIVED,bytes,m_onMessageReceivedParam);
			}

			m_readBufferCount -= (uint16_t) target;
			if (m_readBufferCount)
			{
				for (i=0;i<m_readBufferCount;++i)
				{
					m_readBuffer[i] = m_readBuffer[i+target];
				}
			}

			if ((msgId == 0) || (msgId == rcv->getMessageId()) || (acceptErrorMessage && rcv->getMessageId() == CMT_MID_ERROR))
			{
				m_toEnd = toRestore;
				return m_lastResult = XRV_OK;
			}
		}
		else
		{
			rcv->clear();
			CMT2LOG("L2: waitForMessage load from string failed\n");
		}
	} while (m_toEnd >= getTimeOfDay());

	// a timeout occurred
	if (readsome)
	{
		// check if the current data contains a valid message
		int32_t pos = findValidMessage(m_readBuffer,m_readBufferCount);

		if (pos != -1)
		{
			CMT2LOG("L2: waitForMessage found message in message\n");
			// shift data to start of buffer
			pre = (uint16_t) pos;
			m_readBufferCount -= pre;
			for (i=0;i<m_readBufferCount;++i)
			{
				m_readBuffer[i] = m_readBuffer[i+pre];
			}
			waitForMessage(rcv,msgId,0,acceptErrorMessage);	// parse the message
			m_toEnd = toRestore;
			return m_lastResult;	// set by waitForMessage
		}

		m_lastResult = XRV_TIMEOUT;
	}
	else
		m_lastResult = XRV_TIMEOUTNODATA;
	m_toEnd = toRestore;
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Send a message over the COM port.
XsensResultValue Cmt2s::writeMessage(Message* msg)
{
	CMT2LOG("L2: writeMessage %2x %2x %2x %2x %2x\n",(int32_t) msg->getMessageStart()[0]
													,(int32_t) msg->getMessageStart()[1]
													,(int32_t) msg->getMessageStart()[2]
													,(int32_t) msg->getMessageStart()[3]
													,(int32_t) msg->getMessageStart()[4]);
	uint32_t written = 0;
	m_lastResult = 
			m_cmt1s.writeData(msg->getTotalMessageSize(),msg->getMessageStart(),&written);

	if (m_lastResult != XRV_OK)
	{
		CMT2LOG("L2: writeMessage returns %d: %s\n",(int32_t) m_lastResult, xsensResultText(m_lastResult));
		return m_lastResult;
	}

	if (written != msg->getTotalMessageSize())
	{
		CMT2LOG("L2: writeMessage wrote %u of %u bytes, returns %d: %s\n",written,msg->getTotalMessageSize(),(int32_t) XRV_ERROR, xsensResultText(XRV_ERROR));
		return (m_lastResult = XRV_ERROR);
	}

	if (m_onMessageSent != NULL)
	{
		CmtBinaryData* bytes = (CmtBinaryData*) malloc(sizeof(CmtBinaryData));
		bytes->m_size = msg->getTotalMessageSize();
		bytes->m_portNr = m_cmt1s.getPortNr();
		memcpy(bytes->m_data,msg->getMessageStart(),msg->getTotalMessageSize());
#ifdef _LOG_CALLBACKS
		CMTLOG("C2: m_onMessageSent(%d,(%d,%d),%p)\n",(int32_t) m_onMessageSentInstance, (int32_t) bytes->m_size, (int32_t) bytes->m_portNr, m_onMessageSentParam);
#endif
		m_onMessageSent(m_onMessageSentInstance,CMT_CALLBACK_ONMESSAGESENT,bytes,m_onMessageSentParam);
	}

	CMT2LOG("L2: writeMessage successful\n");
	return (m_lastResult = XRV_OK);
}
	
//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////// Cmt2f  /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////
// Default constructor
Cmt2f::Cmt2f()
{
	m_lastResult = XRV_OK;
	m_readOnly = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Destructor.
Cmt2f::~Cmt2f()
{
	close();
}

//////////////////////////////////////////////////////////////////////////////////////////
// Close the file.
XsensResultValue Cmt2f::close(void)
{
	if (!m_cmt1f.isOpen())
		return m_lastResult = XRV_NOFILEOPEN;

	// save any unsaved data
	// close the file
	m_cmt1f.close();
	m_readOnly = true;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Close the file and delete it.
XsensResultValue Cmt2f::closeAndDelete(void)
{
	if (!m_cmt1f.isOpen())
		return m_lastResult = XRV_NOFILEOPEN;

	// close the file
	m_cmt1f.closeAndDelete();
	m_readOnly = true;
	return m_lastResult = XRV_OK;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Create a new file with level 2 header
XsensResultValue Cmt2f::create(const char* filename)
{
	if (m_cmt1f.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;

	// create file
	m_lastResult = m_cmt1f.create(filename);
	if (m_lastResult != XRV_OK)
		return m_lastResult;

	m_readOnly = false;

	// check if we can actually write to the file
	m_lastResult = m_cmt1f.writeData(5,"Xsens");
	if (m_lastResult == XRV_OK)
		m_lastResult = m_cmt1f.deleteData(0,5);
	if (m_lastResult != XRV_OK)
		m_cmt1f.close();
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Create a new file with level 2 header
XsensResultValue Cmt2f::create(const wchar_t* filename)
{
	if (m_cmt1f.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;

	// create file
	m_lastResult = m_cmt1f.create(filename);
	if (m_lastResult != XRV_OK)
		return m_lastResult;

	m_readOnly = false;

	// check if we can actually write to the file
	m_lastResult = m_cmt1f.writeData(5,"Xsens");
	if (m_lastResult == XRV_OK)
		m_lastResult = m_cmt1f.deleteData(0,5);
	if (m_lastResult != XRV_OK)
		m_cmt1f.close();
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
Cmt1f* Cmt2f::getCmt1f(void)
{
	return &m_cmt1f;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return the error code of the last operation.
XsensResultValue Cmt2f::getLastResult(void) const
{
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the filename that was last successfully opened.
XsensResultValue Cmt2f::getName(char* filename) const
{
	return m_lastResult = m_cmt1f.getName(filename);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Retrieve the filename that was last successfully opened.
XsensResultValue Cmt2f::getName(wchar_t* filename) const
{
	return m_lastResult = m_cmt1f.getName(filename);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Return whether the file is open or not.
bool Cmt2f::isOpen(void) const
{
	return m_cmt1f.isOpen();
}

//////////////////////////////////////////////////////////////////////////////////////////
// Open a file and read the header
XsensResultValue Cmt2f::open(const char* filename, const bool readOnly)
{
	if (m_cmt1f.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;
	m_lastResult = m_cmt1f.open(filename,!readOnly,readOnly);
	m_readOnly = readOnly;
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Open a file and read the header
XsensResultValue Cmt2f::open(const wchar_t* filename, const bool readOnly)
{
	if (m_cmt1f.isOpen())
		return m_lastResult = XRV_ALREADYOPEN;
	m_lastResult = m_cmt1f.open(filename,!readOnly,readOnly);
	m_readOnly = readOnly;
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Read the next message from the file
XsensResultValue Cmt2f::readMessage(Message* msg, const uint8_t msgId)
{
	CmtFilePos pos;
	uint8_t needle = CMT_PREAMBLE;
	uint8_t buffer[CMT_MAXMSGLEN];
	uint32_t length, bcount;
	MessageHeader* hdr = (MessageHeader*) buffer;
	bool extended;
	uint16_t target;

	while (m_lastResult == XRV_OK)
	{
		bcount = 0;

		// find a message preamble
		m_lastResult = m_cmt1f.find(&needle,1,pos);

		if (m_lastResult != XRV_OK)
			return m_lastResult;

		// read header
		m_lastResult = m_cmt1f.readData(CMT_LEN_MSGHEADERCS,buffer,&length);
		bcount += length;
		if (m_lastResult != XRV_OK)
			return m_lastResult;

		if (hdr->m_length == CMT_EXTLENCODE)
		{
			extended = true;
			m_lastResult = m_cmt1f.readData(CMT_LEN_MSGEXTHEADERCS - bcount,&buffer[bcount],&length);
			bcount += length;
			if (m_lastResult != XRV_OK)
			{
				m_cmt1f.setReadPos(pos+1);
				continue;
			}
		}
		else
			extended = false;

		// check the reported size
		if (extended && (((uint16_t) hdr->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) hdr->m_datlen.m_extended.m_length.m_low) > (uint16_t) CMT_MAXDATALEN))
		{
			m_cmt1f.setReadPos(pos+1);
			continue;
		}

		// header seems to be ok, read until end and check checksum
		if (extended)
			target = ((uint16_t) hdr->m_datlen.m_extended.m_length.m_high * 256 + (uint16_t) hdr->m_datlen.m_extended.m_length.m_low) + CMT_LEN_MSGEXTHEADERCS;
		else
			target = hdr->m_length + CMT_LEN_MSGHEADERCS;

		// read the entire message
		m_lastResult = m_cmt1f.readData(target - bcount,&buffer[bcount],&length);
		bcount += length;
		if (m_lastResult != XRV_OK)
		{
			m_cmt1f.setReadPos(pos+1);
			continue;
		}

		// check the checksum
		//msg=new Message(m_readBuffer,(uint16_t) target, (uint16_t) target);
		if (msg->loadFromString(buffer,(uint16_t) target) == XRV_OK)
		{
			if ((msgId == 0) || (msgId == msg->getMessageId()))
				return m_lastResult = XRV_OK;
			pos += target-1;
		}
		msg->clear();
		m_cmt1f.setReadPos(pos+1);
	}
	return m_lastResult;
}

//////////////////////////////////////////////////////////////////////////////////////////
// Get the current file size
CmtFilePos Cmt2f::getFileSize(void)
{
	return m_cmt1f.getFileSize();
}

//////////////////////////////////////////////////////////////////////////////////////////
// Get the current read position of the file
CmtFilePos Cmt2f::getReadPosition(void)
{
	return m_cmt1f.getReadPos();
}

//////////////////////////////////////////////////////////////////////////////////////////
// Set the read position to the given position
XsensResultValue Cmt2f::setReadPosition(CmtFilePos pos)
{
	return m_lastResult = m_cmt1f.setReadPos(pos);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Write a message to the end of the file
XsensResultValue Cmt2f::writeMessage(const Message* msg)
{
	if (m_readOnly)
		return m_lastResult = XRV_READONLY;
	return m_lastResult = m_cmt1f.appendData(msg->getTotalMessageSize(),msg->getMessageStart());
}

} // end of xsens namespace
