
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

#include "devicecommunicator.h"
#include "xsdevice_def.h"

#include <xstypes/xsbusid.h>
#include <xstypes/xsmessage.h>
#include <xstypes/xsfilepos.h>
#include "protocolhandler.h"

#ifdef LOG_COMMUNICATOR_RX_TX
#include <xscommon/xprintf.h>
#endif

/*! \class DeviceCommunicator
	\brief A class that is used for the communcation with a device
*/

/*! \brief Default constructor
*/
DeviceCommunicator::DeviceCommunicator(RxChannelId rxChannels)
	: m_gotoConfigTimeout(m_defaultGotoConfigTimeout)
	, m_nextRxChannelId(0)
#ifdef LOG_COMMUNICATOR_RX_TX_TIMESTAMPED
	, m_logStart(XsTimeStamp::now())
#endif
{
	assert(rxChannels > 0);

	for (XsSize i = 0; i < rxChannels; ++i)
		addRxChannel();

#ifdef LOG_COMMUNICATOR_RX_TX
	generateLogFiles();
#endif
}

/*! \brief Default destructor
*/
DeviceCommunicator::~DeviceCommunicator()
{
}

/*! \brief Write a message and await the reply
	\param msg The message to send
	\param rcv The message to receive
	\param timeout The timeout in ms
	\returns True if successful
*/
bool DeviceCommunicator::doTransaction(const XsMessage &msg, XsMessage &rcv, uint32_t timeout)
{
	XsXbusMessageId expected = static_cast<XsXbusMessageId>(msg.getMessageId() + 1);

	std::shared_ptr<ReplyObject> reply = addReplyObject(expected);
	if (!writeMessage(msg))
	{
		rcv.clear();
		JLALERTG("Failed to write message because " << lastResult() << " " << lastResultText());
		return false;
	}

	rcv = reply->message(timeout);
	if (rcv.getMessageId() == expected)
		return true;

	if (rcv.getMessageId() == XMID_Error)
	{
		setLastResult(static_cast<XsResultValue>(rcv.getDataByte(0)));
		JLALERTG("Received error " << lastResult());
	}
	else
	{
		setLastResult(XRV_TIMEOUT);
		JLALERTG("Timeout waiting for reply to " << msg.getMessageId() << ", timeout = " << timeout << " ms.");
	}

	return false;
}

/*! \brief Does nothing
	\param enable .
*/
void DeviceCommunicator::setKeepAlive(bool enable)
{
	(void)enable;
}

/*! \brief Set the timeout for the gotoConfig function.

	\details The goto config function will try to put the device in config mode, but if the communication
	channel is less than 100% reliable (especially in half-duplex communication like RS485) this
	may fail. For this reason, the function will keep trying until it succeeds or until the timeout
	is reached. Every attempt (even a successful one) will take between 20 and 30ms, so the specified
	timeout is a lower bound for the actual timeout.

	\param timeout The desired timeout value in ms, if 0 the default value is used
*/
void DeviceCommunicator::setGotoConfigTimeout(uint32_t timeout)
{
	if (timeout)
		m_gotoConfigTimeout = timeout;
	else
		m_gotoConfigTimeout = m_defaultGotoConfigTimeout;
}

/*! \copybrief Communicator::getDeviceId
*/
XsResultValue DeviceCommunicator::getDeviceId()
{
	XsMessage snd(XMID_ReqDid);
	snd.setBusId(XS_BID_MASTER);

	XsMessage rcv_did;
	if (!doTransaction(snd, rcv_did))
		return setAndReturnLastResult(XRV_COULDNOTREADSETTINGS);
	uint64_t deviceId = rcv_did.getDataLong();
	if (rcv_did.getDataSize() == 8)
		deviceId = rcv_did.getDataLongLong();

	XsMessage rcv_pdc;
	XsString productCode;
	snd.setMessageId(XMID_ReqProductCode);
	if (doTransaction(snd, rcv_pdc))
	{
		const char* pc = (const char*) rcv_pdc.getDataBuffer();
		std::string result(pc?pc:"", 20);
		std::string::size_type thingy = result.find(" ");
		if (thingy < 20)
			result.erase(result.begin() + (unsigned)thingy, result.end());
		productCode = result;
	}

	XsMessage rcv_hw;
	snd.setMessageId(XMID_ReqHardwareVersion);
	uint16_t hardwareVersion = 0;
	if (doTransaction(snd, rcv_hw))
		hardwareVersion = rcv_hw.getDataShort();

	setMasterDeviceId(XsDeviceId(productCode.c_str(), hardwareVersion, 0, deviceId));

	return XRV_OK;
}

/*! \copybrief Communicator::gotoConfig
*/
XsResultValue DeviceCommunicator::gotoConfig(bool)
{
	XsMessage snd(XMID_GotoConfig), rcv;
	snd.setBusId(XS_BID_MASTER);

	JLDEBUGG("Sending gotoConfig");
	if (!doTransaction(snd, rcv, gotoConfigTimeout()))
	{
		JLALERTG("Failed to go to config, XRV: " << rcv.toResultValue());
		return setAndReturnLastResult(rcv.toResultValue());
	}
	JLDEBUGG("Received gotoConfig ACK");
	return setAndReturnLastResult(XRV_OK);
}

/*! \copybrief Communicator::gotoMeasurement
*/
XsResultValue DeviceCommunicator::gotoMeasurement()
{
	JLDEBUGG("");
	XsMessage snd(XMID_GotoMeasurement);
	snd.setBusId(XS_BID_MASTER);

	if (!doTransaction(snd, defaultTimeout()))
	{
		JLDEBUGG("Transaction failed");
		return setAndReturnLastResult(XRV_CONFIGCHECKFAIL);
	}
	JLDEBUGG("Now in measurement mode");
	return setAndReturnLastResult(XRV_OK);
}

/*! \brief Write \a message to the device
	\param message The message to write
	\returns true on successful write, false otherwise. This doesn't guarantee proper delivery of the message. Use doTransaction for that.
*/
bool DeviceCommunicator::writeMessage(const XsMessage &message)
{
	XsByteArray raw;
	if (ProtocolHandler::composeMessage(raw, message) < 0)
	{
		setLastResult(XRV_INVALIDMSG);
		return false;
	}
	JLTRACEG("did: " << masterDeviceId() << " writing message " << message.toHexString(10));
	setLastResult(writeRawData(raw));
	if (lastResult() == XRV_OK)
	{
#ifdef LOG_COMMUNICATOR_RX_TX
		logTxStream(message);
#endif
		if (masterDevice() != nullptr)
			masterDevice()->onMessageSent(message);
		return true;
	}
	return false;
}

/*! \brief Adds an RX (receive) channel to the device communicator.
	Each channel maintains its own message parsing state
	\returns The id of the added channel
*/
DeviceCommunicator::RxChannelId DeviceCommunicator::addRxChannel()
{
	RxChannelId channel = m_nextRxChannelId++;
	m_messageExtractors.push_back(MessageExtractor{protocolManager()});
	return channel;
}

/*! \brief Read all messages available in the incoming data stream after adding new data supplied in \a rawIn
	\details This function will read all messages present in the DeviceCommunicator's message extracting buffer after
	appending it with the newly received data.

	\param[in] rawIn the newly incoming data
	\param[out] messages the list of messages that was extracted
	\param[in] channel the channel to extract from
	\returns XRV_OK on success, something else on failure
*/
XsResultValue DeviceCommunicator::extractMessages(const XsByteArray &rawIn, std::deque<XsMessage>& messages, RxChannelId channel)
{
	if (channel >= m_messageExtractors.size())
		return XRV_ERROR;

	assert(protocolManager());

	XsResultValue res = m_messageExtractors[channel].processNewData(masterDevice(), rawIn, messages);

	if (res == XRV_OK)
	{
		for (auto const& msg : messages)
		{
			if (masterDevice() != nullptr)
				masterDevice()->onMessageReceived(msg);
		}
	}
	return res;
}

/*! \brief Returns the message extractor for the given rx channel
*/
MessageExtractor& DeviceCommunicator::messageExtractor(RxChannelId channel)
{
	assert(channel < m_messageExtractors.size());
	return m_messageExtractors[channel];
}

/*! \copybrief Communicator::handleMessage
*/
void DeviceCommunicator::handleMessage(const XsMessage &message)
{
#ifdef LOG_COMMUNICATOR_RX_TX
	logRxStream(message);
#endif
	Communicator::handleMessage(message);
}

/* Now the disabled stuff starts */

//! \cond DO_NOT_DOCUMENT
void DeviceCommunicator::closeLogFile()
{
}

XsMessage DeviceCommunicator::readMessageFromStartOfFile(uint8_t, int)
{
	return XsMessage();
}

std::deque<XsMessage> DeviceCommunicator::readMessagesFromStartOfFile(uint8_t, int)
{
	return std::deque<XsMessage>();
}

void DeviceCommunicator::loadLogFile(XsDevice*)
{
}

XsResultValue DeviceCommunicator::readLogFile(XsDevice*)
{
	return XRV_UNSUPPORTED;
}

XsResultValue DeviceCommunicator::readSinglePacketFromFile()
{
	return XRV_UNSUPPORTED;
}

void DeviceCommunicator::abortLoadLogFile()
{
}

bool DeviceCommunicator::openLogFile(const XsString &)
{
	return false;
}

XsString DeviceCommunicator::logFileName() const
{
	return XsString();
}

XsFilePos DeviceCommunicator::logFileSize() const
{
	return 0;
}

XsTimeStamp DeviceCommunicator::logFileDate() const
{
	return 0;
}

XsFilePos DeviceCommunicator::logFileReadPosition() const
{
	return XsFilePos(0);
}

void DeviceCommunicator::resetLogFileReadPosition(void)
{
}

bool DeviceCommunicator::isReadingFromFile() const
{
	return false;
}

void DeviceCommunicator::waitForLastTaskCompletion()
{
}

XsMessage DeviceCommunicator::readMessage(uint8_t)
{
	return XsMessage();
}

//! \endcond

#ifdef LOG_COMMUNICATOR_RX_TX
void DeviceCommunicator::generateLogFiles()
{
	XsString date = XsTime::getDateAsString();
	XsString time = XsTime::getTimeAsString();

#ifdef LOG_COMMUNICATOR_RX_TX_TIMESTAMPED
	XsString extension("mts");
#else
	XsString extension("mtb");
#endif

	auto generateFilename = [&](XsString const& stream)
	{
		std::string temp = xprintf("communicator_%s_%s_%s.%s", stream.c_str(), date.c_str(), time.c_str(), extension.c_str());
		return XsString(temp);
	};

	m_rxLog.create(generateFilename("rx"), false);
	m_txLog.create(generateFilename("tx"), false);
}

void DeviceCommunicator::logTxStream(XsMessage const& msg)
{
	assert(m_txLog.isOpen());

#ifdef LOG_COMMUNICATOR_RX_TX_TIMESTAMPED
	XsTimeStamp diff = XsTimeStamp::now() - m_logStart;
	std::string timestamp = xprintf("%d ", diff.msTime());
	m_txLog.write(timestamp.c_str(), 1, timestamp.length());
#endif

	m_txLog.write(msg.rawMessage().data(), 1, msg.rawMessage().size());
	m_txLog.flush();
}

void DeviceCommunicator::logRxStream(XsMessage const& msg)
{
	assert(m_rxLog.isOpen());

#ifdef LOG_COMMUNICATOR_RX_TX_TIMESTAMPED
	XsTimeStamp diff = XsTimeStamp::now() - m_logStart;
	std::string timestamp = xprintf("%d ", diff.msTime());
	m_rxLog.write(timestamp.c_str(), 1, timestamp.length());
#endif

	m_rxLog.write(msg.rawMessage().data(), 1, msg.rawMessage().size());
	m_rxLog.flush();
}

#endif
