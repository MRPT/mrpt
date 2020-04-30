
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

#include "dataparser.h"
#include "xscontrollerconfig.h"
#include <xstypes/xsmessage.h>


/*!	\class DataParser
	\brief A class for the data parsing on a separete thread
*/

/*! \brief Default constructor
*/
DataParser::DataParser()
{
	startThread();
}

/*! Default destructor
*/
DataParser::~DataParser()
{
	terminate();
}

/*! \brief Adds the raw data to an array
	\param arr The reference to a byte array to which the data will be added
*/
void DataParser::addRawData(const XsByteArray &arr)
{
	xsens::Lock locky(&m_incomingMutex);
	m_incoming.push(arr);
	locky.unlock();
	m_newDataEvent.set();
}

/*! \brief The inner thread function
*/
int32_t DataParser::innerFunction()
{
	// wait for new data
	if (!m_newDataEvent.wait())
		return 0;

	// get new data
	XsByteArray raw;
	xsens::Lock lockIncoming(&m_incomingMutex);
	while (!m_incoming.empty() && !isTerminating())
	{
		raw.append(m_incoming.front());
		m_incoming.pop();
		lockIncoming.unlock();

		JLTRACEG("raw size: " << raw.size());

		// process data
		if (!raw.empty() && !isTerminating())
		{
			std::deque<XsMessage> msgs;
			XsResultValue res = processBufferedData(raw, msgs);
			JLTRACEG("Parse result " << res << ": " << msgs.size() << " messages");

			if (res != XRV_TIMEOUT && res != XRV_TIMEOUTNODATA && !isTerminating())
			{
				for (XsMessage const& msg : msgs)
				{
					handleMessage(msg);
					if (isTerminating())
						break;
				}
			}
			raw.clear();
		}

		lockIncoming.lock();
	}
	m_newDataEvent.reset();

	return 1;
}

/*! \brief Initializes the thread
*/
void DataParser::initFunction()
{
	setPriority(XS_THREAD_PRIORITY_HIGH);
	sprintf(m_parserType, "XDA %s %p", parserType(), this);
	//JLDEBUGG("Thread " << this << " " << buffer);
	xsNameThisThread(m_parserType);
}

/*! \brief Clears the data queue
*/
void DataParser::clear()
{
	xsens::Lock lockIncoming(&m_incomingMutex);

	while (!m_incoming.empty())
		m_incoming.pop();
}

void DataParser::signalStopThread(void)
{
	StandardThread::signalStopThread();
	m_newDataEvent.terminate();
}

/*! \brief Terminates the thread
*/
void DataParser::terminate()
{
	JLDEBUGG("Thread " << this << " type: " << m_parserType);
	stopThread();
	clear();
}
