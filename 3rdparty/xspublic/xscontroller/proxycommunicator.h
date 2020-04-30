
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

#ifndef PROXYCOMMUNICATOR_H
#define PROXYCOMMUNICATOR_H

#include "serialcommunicator.h"
#include "dataparser.h"
#include <xstypes/xsportinfo.h>

class CallbackManagerXda;
class ProxyCommunicator : public SerialCommunicator
{
public:
	ProxyCommunicator(int channelId, uint32_t channelLatency);
	~ProxyCommunicator() override;

	XsResultValue readDataToBuffer(XsByteArray& raw) override;
	XsResultValue processBufferedData(const XsByteArray& rawIn, std::deque<XsMessage>& messages) override;
	void handleMessage(const XsMessage &message) override;

	void flushPort() override;
	void closePort() override;
	bool isPortOpen() const override;
	XsPortInfo portInfo() const override;
	bool openPort(const XsPortInfo &portInfo, OpenPortStage stage = OPS_Full, bool detectRs485 = false) override;
	bool reopenPort(OpenPortStage stage = OPS_Full, bool skipDeviceIdCheck = false) override;
	bool isDockedAt(Communicator *other) const override;
	bool allowReprocessing() const override { return false; }

	void handleReceivedData(const XsByteArray& data);

	bool doTransaction(const XsMessage &msg, XsMessage &rcv, uint32_t timeout) override;

	static XsPortInfo createPortInfo(int channelId);
protected:
	XsResultValue writeRawData(const XsByteArray &data) override;
	std::shared_ptr<StreamInterface> createStreamInterface(const XsPortInfo &pi) override { (void)pi; return std::shared_ptr<StreamInterface>(); }

	uint32_t defaultInterfaceTimeout() const override;

private:
	int m_channelId;
	XsPortInfo m_activePortInfo;
	uint32_t m_channelLatency;

};

#endif
