
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

#include "proxycommunicator.h"
#include "callbackmanagerxda.h"
#include <xstypes/xsportinfo.h>
#include <xscommon/xsens_janitors.h>
#include "deviceredetector.h"


/*! \class ProxyCommunicator

	The ProxyCommunicator is a communicator that uses an external user defined communication channel. Any data
	to be transmitted by XDA to a device is given to the user in a callback (\a onTransmissionRequest) and it is
	up to the user how that data is transported. Data coming back from a device is inserted into the ProxyCommunicator
	by calling the \a handleReceivedData function. There can be multiple ProxyCommunicators as long as their user-provided
	channel identifiers are different.
*/

/*! \brief Constructor
	\param channelId The user-provided identifier associated with this communicator
	\param channelLatency The latency of channel
*/
ProxyCommunicator::ProxyCommunicator(int channelId, uint32_t channelLatency)
	: m_channelId (channelId)
	, m_channelLatency(channelLatency)
{
}

/*! \brief Destructor
*/
ProxyCommunicator::~ProxyCommunicator()
{
}

/*! \brief Flushes all remaining data on the open port. Has no effect for the ProxyCommunicator
*/
void ProxyCommunicator::flushPort()
{
	//ProxyCommunicator maintains no stream. No action required
}

/*! \brief Closes the port
*/
void ProxyCommunicator::closePort()
{
	m_activePortInfo = XsPortInfo();
}

/*! \brief Checks if the associated port is open
*/
bool ProxyCommunicator::isPortOpen() const
{
	return !m_activePortInfo.portName().empty();
}

/*! \returns port information
*/
XsPortInfo ProxyCommunicator::portInfo() const
{
	return m_activePortInfo;
}

/*! \brief Opens a proxy port
	\param portInfo The port information
	\param stage The open port stage
	\param detectRs485 Enable more extended scan to detect rs485 devices
	\details If successfull information on the connected device is available in the port info \sa portInfo
	\returns True if successful
*/
bool ProxyCommunicator::openPort(const XsPortInfo &portInfo, OpenPortStage stage, bool detectRs485)
{
	auto result = [&]()
	{
		bool ok = (lastResult() == XRV_OK);
		if (!ok)
			prepareForDestruction();
		return ok;
	};

	if (stage & OPS_OpenPort)
	{
		if (isPortOpen())
		{
			setLastResult(XRV_ALREADYOPEN);
			return (this->portInfo() == portInfo);
		}
	}

	if (stage & OPS_InitDevice)
	{
		if (gotoConfig(detectRs485) != XRV_OK)
			return result();

		if (getDeviceId() != XRV_OK)
			return result();
	}

	setLastResult(XRV_OK);
	m_activePortInfo = portInfo;
	m_activePortInfo.setDeviceId(masterDeviceId());
	return result();
}

/*! \brief Closes and tries to reopen the port
	\param stage The current stage of port
	\param skipDeviceIdCheck If set to true the it will skip device id check
	\returns true if the the port could be successfully reopened
*/
bool ProxyCommunicator::reopenPort(OpenPortStage stage, bool skipDeviceIdCheck)
{
	XsPortInfo pi = portInfo();

	closePort();
	DeviceRedetector redetector(pi);
	if (!redetector.redetect(masterDeviceId(), pi, skipDeviceIdCheck))
		return false;

	return openPort(pi, stage);
}

/*! \brief Has no effect for the ProxyCommunicator. Always returns false
*/
bool ProxyCommunicator::isDockedAt(Communicator *) const
{
	return false;
}

/*! \brief Writes raw data to the communication channel
*/
XsResultValue ProxyCommunicator::writeRawData(const XsByteArray &data)
{
	onTransmissionRequest(m_channelId, &data);
	return XRV_OK;
}

/*! \brief Handles data received from the communication channel
*/
void ProxyCommunicator::handleReceivedData(const XsByteArray& data)
{
	addRawData(data);
}

/*! \brief Write a message and await the reply */
bool ProxyCommunicator::doTransaction(const XsMessage &msg, XsMessage &rcv, uint32_t timeout)
{
	return SerialCommunicator::doTransaction(msg, rcv, timeout + m_channelLatency);
}

/*! \brief Has no effect for the ProxyCommunicator
*/
XsResultValue ProxyCommunicator::readDataToBuffer(XsByteArray& )
{
	//No action required as data is pushed (no polling required) in the buffer by the handleReceivedData function
	return XRV_OK;
}

/*! \brief Handles a message received on the communication channel
*/
void ProxyCommunicator::handleMessage(const XsMessage &msg)
{
	DeviceCommunicator::handleMessage(msg);
}

/*! \brief Read all messages from the buffered read data after adding new data supplied in \a rawIn
	\param rawIn The byte array with all data
	\param messages The message to process
	\returns The messages that were read.
*/
XsResultValue ProxyCommunicator::processBufferedData(const XsByteArray& rawIn, std::deque<XsMessage>& messages)
{
	return extractMessages(rawIn, messages);
}

/*! \brief Creates a default port info object based on the given user-provided channel identifier
*/
XsPortInfo ProxyCommunicator::createPortInfo(int channelId)
{
	XsPortInfo portInfo;
	char tmp[16];
	snprintf(tmp, sizeof(tmp), "PROXY#%d", channelId);
	portInfo.setPortName(tmp);
	return portInfo;
}

/*! \brief Returns the default timeout needed for this interface
*/
uint32_t ProxyCommunicator::defaultInterfaceTimeout() const
{
	return m_channelLatency;
}
