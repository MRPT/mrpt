
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


#include "mtthread.h"

#include "serialcommunicator.h"
#include <xstypes/xsmessage.h>
#include <xstypes/xsbusid.h>

using namespace xsens;

/*! \class MtThread
	\brief A class that implements thread for MT
*/

/*! \brief Default constructor
*/
MtThread::MtThread(DataParser& fetcher, SerialCommunicator& communicator)
	: DataPoller(fetcher)
	, m_doGotoConfig(false)
	, m_communicator(&communicator)
	, m_gotoConfigPlus(0)
{
	XsMessage gotoConfig(XMID_GotoConfig);
	XsMessage largeGotoConfig(XMID_GotoConfig, 30);
	m_gotoConfigPlus = new XsByteArray(largeGotoConfig.rawMessage());
	for (int i = 0; i < 6; ++i)
		m_gotoConfigPlus->append(gotoConfig.rawMessage());
}

/*! \brief Default destructor
*/
MtThread::~MtThread(void)
{
	cleanup();
	if (m_gotoConfigPlus)
		delete m_gotoConfigPlus;
}

/*! \brief Set whether we should send gotoconfig here
*/
void MtThread::setDoGotoConfig(bool doit)
{
	srand( (unsigned int)XsTime_timeStampNow(0));
	m_doGotoConfig = doit;
}

/*! \brief The inner thread function
	\details This function handles port communication, delegating processing and calibration to its DataParser.
	\returns A value from 0 to 3
*/
int32_t MtThread::innerFunction(void)
{
	if (m_doGotoConfig)
	{
		JLDEBUGG("Sending gotoConfig");

		if (m_communicator->writeRawData(*m_gotoConfigPlus) != XRV_OK)
			JLALERTG("Sending gotoConfig failed");
		XsTime_msleep(((long)rand() * 10)/RAND_MAX+5);	// if we sent a goto config, wait a bit for the result
	}

	return DataPoller::innerFunction();
}
