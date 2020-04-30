
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

#ifndef DEVICECOMMUNICATOR_H
#define DEVICECOMMUNICATOR_H

#include "communicator.h"
#include "messageextractor.h"

#ifdef LOG_COMMUNICATOR_RX_TX
#include <xstypes/xsfile.h>
#endif


class DeviceCommunicator : public Communicator
{
public:

	//! \brief A typedef for Rx chanel ID
	typedef XsSize RxChannelId;

	DeviceCommunicator(RxChannelId rxChannels = 1);

	// live stuff
	XsResultValue getDeviceId() override;
	XsResultValue gotoConfig(bool) override;
	XsResultValue gotoMeasurement() override;

	//! \returns The timeout value for gotoConfig function
	uint32_t gotoConfigTimeout() const { return m_gotoConfigTimeout; }
	void setGotoConfigTimeout(uint32_t timeout) override;
	bool writeMessage(const XsMessage &message) override;

	void handleMessage(const XsMessage &message) override;

	using Communicator::doTransaction;
	virtual bool doTransaction(const XsMessage &msg, XsMessage &rcv, uint32_t timeout) override;

	void setKeepAlive(bool enable) override;

	// file stuff
	void closeLogFile() override;
	XsMessage readMessage(uint8_t msgId = 0) override;
	XsMessage readMessageFromStartOfFile(uint8_t msgId, int maxMsgs = 0) override;
	std::deque<XsMessage> readMessagesFromStartOfFile(uint8_t msgId, int maxMsgs = 0) override;
	void loadLogFile(XsDevice* device) override;
	void abortLoadLogFile() override;
	bool openLogFile(const XsString &filename) override;
	XsString logFileName() const override;
	XsFilePos logFileSize() const override;
	XsTimeStamp logFileDate() const override;
	XsFilePos logFileReadPosition() const override;
	void resetLogFileReadPosition(void) override;
	bool isReadingFromFile() const override;
	void waitForLastTaskCompletion() override;

	/*! \brief Read a log file into cache
		\param device The device to read log from
		\returns XRV_OK if successful
	*/
	virtual XsResultValue readLogFile(XsDevice* device);

	/*! \brief Read a single XsDataPacket from an open log file
		\returns XRV_OK if successful
	*/
	virtual XsResultValue readSinglePacketFromFile();

protected:
	~DeviceCommunicator() override;
	XsResultValue extractMessages(const XsByteArray &rawIn, std::deque<XsMessage> &messages, RxChannelId channel = 0);

	/*! \brief Writes a raw data to a device
		\param data The raw data to write
		\return XRV_OK if successful
	*/
	virtual XsResultValue writeRawData(const XsByteArray &data) = 0;
	RxChannelId addRxChannel();
	MessageExtractor& messageExtractor(RxChannelId = 0);

#ifdef LOG_COMMUNICATOR_RX_TX
	void logTxStream(XsMessage const& msg);
	void logRxStream(XsMessage const& msg);
#endif

private:
	static const uint32_t m_defaultGotoConfigTimeout = 150;	// 1500 just after powerup, 100ms is not enough sometimes during tests

	uint32_t m_gotoConfigTimeout;

	RxChannelId m_nextRxChannelId;
	std::vector<MessageExtractor> m_messageExtractors;

#ifdef LOG_COMMUNICATOR_RX_TX
	XsFile m_rxLog;
	XsFile m_txLog;
#ifdef LOG_COMMUNICATOR_RX_TX_TIMESTAMPED
	XsTimeStamp m_logStart;
#endif

	void generateLogFiles();
#endif
};

#endif
