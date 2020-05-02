
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

#ifndef DATAPARSER_H
#define DATAPARSER_H

#include <deque>
#include <xstypes/xsresultvalue.h>
#include <xscommon/threading.h>
#include <xstypes/xsbytearray.h>
#include <queue>

struct XsMessage;

class DataParser : protected xsens::StandardThread
{
public:
	DataParser();
	~DataParser() override;

	/*! \brief Read available data from the open IO device
		\param raw A buffer that will receive the read data.
		\returns XRV_OK if successful
	*/
	virtual XsResultValue readDataToBuffer(XsByteArray& raw) = 0;

	/*! \brief Read all messages from the buffered read data after adding new data supplied in \a rawIn
		\param rawIn The byte array with all data
		\param messages The message to process
		\returns The messages that were read.
	*/
	virtual XsResultValue processBufferedData(const XsByteArray& rawIn, std::deque<XsMessage>& messages) = 0;

	//! \copybrief Communicator::handleMessage
	virtual void handleMessage(const XsMessage &message) = 0;

	void addRawData(const XsByteArray &arr);
	void clear();
	void terminate();

	//! \returns The parser type
	virtual const char* parserType() const { return "DataParser"; }

protected:
	void initFunction() override;
	int32_t innerFunction() override;
	void signalStopThread(void) override;

private:
	xsens::Mutex m_incomingMutex;
	std::queue<XsByteArray> m_incoming;
	xsens::WaitEvent m_newDataEvent;
	char m_parserType[128];
};

#endif
