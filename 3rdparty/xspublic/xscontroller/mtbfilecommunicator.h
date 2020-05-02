
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

#ifndef MTBFILECOMMUNICATOR_H
#define MTBFILECOMMUNICATOR_H

#include "communicator.h"
#include "fileloader.h"

#include <xscommon/xsens_threadpool.h>
#include <xstypes/xsresultvalue.h>

#include <memory>

class IoInterfaceFile;
class MessageExtractor;

class MtbFileCommunicator : public Communicator, protected FileLoader
{
public:
	static Communicator *construct();
	MtbFileCommunicator();

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
	bool isLoadLogFileInProgress() const override;

	bool doTransaction(const XsMessage &msg, XsMessage &rcv, uint32_t timeout) override;

	XsResultValue gotoConfig(bool detectRs485 = false) override;
	XsResultValue gotoMeasurement() override;
	XsResultValue getDeviceId() override;
	void setGotoConfigTimeout(uint32_t timeout) override;
	bool writeMessage(const XsMessage &message) override;
	void flushPort() override;
	void closePort() override;
	bool isPortOpen() const override;
	XsPortInfo portInfo() const override;
	bool openPort(const XsPortInfo &portInfo, OpenPortStage stage = OPS_Full, bool detectRs485 = false) override;
	bool reopenPort(OpenPortStage stage = OPS_Full, bool skipDeviceIdCheck = false) override;
	bool isDockedAt(Communicator *other) const override;
	void setKeepAlive(bool enable) override;
	void addProtocolHandler(IProtocolHandler *handler) override;

protected:
	MtbFileCommunicator(std::shared_ptr<IoInterfaceFile> ioInterfaceFile);
	~MtbFileCommunicator();
	void prepareForDestruction() override;

	virtual XsResultValue readLogFile(XsDevice* device) override;
	virtual XsResultValue readSinglePacketFromFile() override;

	void waitForLastTaskCompletion() override;

	virtual XsMessage readNextMessage();

private:
	uint32_t timeoutToMaxMessages(uint32_t timeout);
	void completeAllThreadedWork();

	std::shared_ptr<IoInterfaceFile> m_ioInterfaceFile;
	bool m_abortLoadLogFile;
	xsens::ThreadPool::TaskId m_loadFileTaskId;

	MessageExtractor* m_extractor;
	std::deque<XsMessage>* m_extractedMessages;
};

#endif
