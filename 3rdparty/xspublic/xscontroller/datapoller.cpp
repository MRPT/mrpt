
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

#include "datapoller.h"
#include "dataparser.h"

/*! \brief Create a DataPoller with a \a parser */

DataPoller::DataPoller(DataParser& parser)
	: m_parser(parser)
{
}

/*! \brief Destroy the data poller */

DataPoller::~DataPoller()
{
	cleanup();
}

/*! \brief Returns the DataParser to be used for parsing the read data */
DataParser& DataPoller::dataParser() const
{
	return m_parser;
}

/*! \brief Conjure up the time to wait based on properties of the received data (like the length) */
int32_t DataPoller::conjureUpWaitTime(const XsByteArray &bytes) const
{
	if (bytes.size() == 0)
		return 3;
	else if (bytes.size() < 256)
		return 2;
	return 0;
}

/*! \brief Init function for the thread, sets the priority higher
*/

void DataPoller::initFunction()
{
	setPriority(XS_THREAD_PRIORITY_HIGHER);
	char buffer[128];
	sprintf(buffer, "XDA %s Poller %p", m_parser.parserType(), &m_parser);
	xsNameThisThread(buffer);
}

/*! \brief Clean up the DataPoller */

void DataPoller::cleanup()
{
	assert(getThreadId() != xsGetCurrentThreadId());
	stopThread();
}

/*! \brief The inner thread function
*/
int32_t DataPoller::innerFunction(void)
{
	XsByteArray ba;
	if (m_parser.readDataToBuffer(ba) != XRV_OK)
		return 1;

	int32_t retval = conjureUpWaitTime(ba);
	if (ba.size())
		m_parser.addRawData(ba);

	return retval;
}
