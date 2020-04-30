
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

#include "serialcommunicator.h"
#include <xstypes/xsbusid.h>
#include <xstypes/xsexception.h>
#include "protocolhandler.h"
#include "serialinterface.h"
#include <xstypes/xsportinfoarray.h>
#include <xstypes/xsdid.h>
#include <xstypes/xstime.h>
#include "xsscanner.h"
#include "mtthread.h"
#include "deviceredetector.h"
#include <xstypes/xsversion.h>
#include "xsdevice_def.h"


/*! \class SerialCommunicator
	\brief A class that uses serial communication
*/

/*! \brief Default constructor
*/
SerialCommunicator::SerialCommunicator()
	: m_thread(*this, *this)
	, m_firmwareRevision(0,0,0)
	, m_hardwareRevision(0,0)
{
	messageExtractor().clearBuffer();
	startPollThread();
}

/*! \brief Default destructor
*/
SerialCommunicator::~SerialCommunicator()
{
}

/*! \brief Prepares for a destruction
*/
void SerialCommunicator::prepareForDestruction()
{
	JLDEBUGG("");

	stopPollThread();
	terminate();

	messageExtractor().clearBuffer();
	if (m_streamInterface)
	{
		m_streamInterface->cancelIo();
		m_streamInterface->close();
		m_streamInterface.reset();
	}

	DeviceCommunicator::prepareForDestruction();
}

/*! \copybrief Communicator::gotoMeasurement
*/
XsResultValue SerialCommunicator::gotoMeasurement()
{
	JLDEBUGG("entry");

	if (!isActive())
		return setAndReturnLastResult(XRV_INVALIDOPERATION);

	setDoGotoConfig(false);

	XsResultValue r = DeviceCommunicator::gotoMeasurement();
	if (r != XRV_OK)
		gotoConfig();
	return setAndReturnLastResult(r);
}

/*! \copybrief Communicator::gotoConfig
*/
XsResultValue SerialCommunicator::gotoConfig(bool detectRs485)
{
	JLDEBUGG("entry");

	bool rs485Config = ((masterDeviceId().toInt() & XS_DID_TYPEL_COMM_MASK) == XS_DID_TYPEL_RS485) || detectRs485;
	if (!rs485Config)
		return DeviceCommunicator::gotoConfig(false);

	uint32_t interfaceTimeout = defaultInterfaceTimeout();
	if (m_streamInterface)
		interfaceTimeout = m_streamInterface->getTimeout();

	std::shared_ptr<ReplyObject> reply = addReplyObject(XMID_GotoConfigAck);

	int oldCount = messageExtractor().setMaxIncompleteRetryCount(0);
	setDoGotoConfig(true);
	XsMessage rcv = reply->message(gotoConfigTimeout() + interfaceTimeout + 1000); // set higher timeout for RS485
	setDoGotoConfig(false);

	if (rcv.getMessageId() == XMID_GotoConfigAck)
	{
		// wait a bit longer for all potentially incoming gotoConfigAcks to be received and processed
		while (true)
		{
			std::shared_ptr<ReplyObject> reply2 = addReplyObject(XMID_GotoConfigAck);
			XsMessage rcv = reply2->message(100);
			if (rcv.getMessageId() != XMID_GotoConfigAck)
				break;
		}

		messageExtractor().setMaxIncompleteRetryCount(oldCount);
		JLTRACEG("Ok: " << rcv.getMessageId());
		return (setAndReturnLastResult(XRV_OK));
	}

	messageExtractor().setMaxIncompleteRetryCount(oldCount);
	JLDEBUGG("Fail: " << rcv.getMessageId());
	return (setAndReturnLastResult(XRV_CONFIGCHECKFAIL));
}

/*! \brief Write raw data to the open COM or USB port
*/
XsResultValue SerialCommunicator::writeRawData(const XsByteArray &data)
{
	if (!isPortOpen())
		return XRV_NOPORTOPEN;
	return m_streamInterface->writeData(data);
}

/*! \brief Flushes all remaining data on the open port
*/
void SerialCommunicator::flushPort()
{
	if (m_streamInterface)
		m_streamInterface->flushData();
}

/*! \brief Stops polling the thread
*/
void SerialCommunicator::stopPollThread()
{
	m_thread.stopThread();
}

/*! \brief Starts polling the thread
*/
void SerialCommunicator::startPollThread()
{
	m_thread.startThread();
}
/*! \brief Closes the port
*/
void SerialCommunicator::closePort()
{
	JLDEBUGG("entry");
	XSEXITLOGN(gJournal);

	if (!m_streamInterface)
		return;

	stopPollThread();

	if (m_streamInterface)
	{
		m_streamInterface->close();
		m_streamInterface.reset();
	}
}

/*! \returns whether the communicator has an open COM or USB port
\*/
bool SerialCommunicator::isPortOpen() const
{
	return m_streamInterface && m_streamInterface->isOpen();
}

/*! \returns The port information
*/
XsPortInfo SerialCommunicator::portInfo() const
{
	return m_activePortInfo;
}

/*! \brief Open a serial port and return the main device connected to it
*/
bool SerialCommunicator::openPort(const XsPortInfo &portInfo, OpenPortStage stage, bool detectRs485)
{
	JLDEBUGG("Opening " << portInfo << " stage " << stage << " configtimeout " << gotoConfigTimeout());
	XSEXITLOGN(gJournal);

	if (stage & OPS_OpenPort)
	{
		if (m_streamInterface)
		{
			setLastResult(XRV_ALREADYOPEN);
			return (this->portInfo() == portInfo);
		}

		m_streamInterface = createStreamInterface(portInfo);

		if (lastResult() != XRV_OK)
		{
			stopPollThread();	// it's possible that the poll thread has been started, but the result is still not ok
			m_streamInterface.reset();
			return false;
		}
		startPollThread();
	}
	if (!m_streamInterface)
	{
		setLastResult(XRV_NOPORTOPEN);
		return false;
	}

	if (stage & OPS_InitDevice)
	{
		// Retrieve deviceid, which is always needed with an open port
		if (gotoConfig(detectRs485) != XRV_OK)
		{
			prepareForDestruction();
			return false;
		}

		if (getDeviceId() != XRV_OK)
		{
			prepareForDestruction();
			return false;
		}

		// Also retrieve the firmware revision, which may be needed for determining optional settings of a port
		if (getFirmwareRevision() != XRV_OK)
		{
			prepareForDestruction();
			return false;
		}

		// Get the hardware version only for Awinda Dongle. Issue: MTSDK-3999
		if (masterDeviceId().isAwinda2Dongle())
		{
			if (getHardwareRevision() != XRV_OK)
			{
				prepareForDestruction();
				return false;
			}
		}
	}

	setLastResult(m_streamInterface->setTimeout(0));

	if (lastResult() == XRV_OK)
	{
		m_activePortInfo = portInfo;
		return true;
	}
	else
	{
		return false;
	}

}

/*! \copybrief Communicator::reopenPort
*/
bool SerialCommunicator::reopenPort(OpenPortStage stage, bool skipDeviceIdCheck)
{
	JLDEBUGG("");
	XsPortInfo pi = portInfo();

	uint32_t oldTimeOut = defaultTimeout() + defaultInterfaceTimeout();
	if (m_streamInterface)
		oldTimeOut = m_streamInterface->getTimeout();

	closePort();
	DeviceRedetector redetector(pi);
	bool detected = false;
	int count = 0;
	while (!detected && count < 3)
	{
		if (!redetector.redetect(masterDeviceId(), pi, skipDeviceIdCheck))
		{
			JLDEBUGG("Redetect failed");
			return false;
		}

		if (!openPort(pi, stage))
		{
			JLDEBUGG("openPort failed, attempt: " << count);
			count++;
			continue;
		}
		detected = true;
	}
	if (!detected)
		return false;

	if (oldTimeOut && m_streamInterface)
		m_streamInterface->setTimeout(oldTimeOut);

	return true;
}

/*! \returns True if the thread is alive*/
bool SerialCommunicator::isActive() const
{
	return ((masterDevice() != nullptr) && (m_thread.isAlive()));
}

/*! \brief Sets do go to config in a thread
	\param doit The boolean value to set
*/
void SerialCommunicator::setDoGotoConfig(bool doit)
{
	m_thread.setDoGotoConfig(doit);
}


/*! \returns True if the \a other device is docked at this device
	\param other The communicator of an other device
	\note This is true if the USB hub matches.
*/
bool SerialCommunicator::isDockedAt(Communicator *other) const
{
	XsUsbHubInfo thisHub = XsScanner::scanUsbHub(XsPortInfo(portInfo().portName()));
	XsUsbHubInfo otherHub = XsScanner::scanUsbHub(XsPortInfo(other->portInfo().portName()));
	return thisHub.parentPathMatches(otherHub);
}

/*! \brief Read available data from the open IO device
	\details This function will attempt to read all available data from the open device (COM port
	or USB port).
	The function will read from the device, but it won't wait for data to become available.
	\param raw A buffer that will receive the read data.
	\note This function does not set the lastResult value since this could cause threading issues.
	\returns The result of the operation
*/
XsResultValue SerialCommunicator::readDataToBuffer(XsByteArray& raw)
{
	if (!m_streamInterface)
		return XRV_NOPORTOPEN;

	// always read data and append it to the cache before doing analysis
	const int maxSz = 8192;
	XsResultValue res = m_streamInterface->readData(maxSz, raw);
	if (raw.size())
		return XRV_OK;

	switch (res)
	{
	// all intended fall-throughs
	case XRV_UNEXPECTED_DISCONNECT:
		if (masterDevice() != nullptr)
			masterDevice()->onConnectionLost();
		// Fallthrough.
	case XRV_NOFILEORPORTOPEN:
		closePort();
	default:
		break;
	}

	return res;
}

/*! \copybrief Communicator::handleMessage
	\note Overridden here for implementation of DataParser::handleMessage
	\param msg The XsMessage to handle
*/
void SerialCommunicator::handleMessage(const XsMessage &msg)
{
	DeviceCommunicator::handleMessage(msg);
}

/*! \brief Read all messages from the buffered read data after adding new data supplied in \a rawIn
	\param rawIn The byte array with all data
	\param messages The message to process
	\details This function will read all present messages in the read buffer. In order for this function
	to work, you need to call readDataToBuffer() first.
	\returns The messages that were read.
 */
XsResultValue SerialCommunicator::processBufferedData(const XsByteArray& rawIn, std::deque<XsMessage>& messages)
{
	return extractMessages(rawIn, messages);
}

/*!	\brief Requests the firmware revision from the connected device
*/
XsResultValue SerialCommunicator::getFirmwareRevision()
{
	XsMessage snd(XMID_ReqFirmwareRevision);
	snd.setBusId(XS_BID_MASTER);

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return setAndReturnLastResult(XRV_COULDNOTREADSETTINGS);
	m_firmwareRevision = XsVersion(rcv.getDataByte(0), rcv.getDataByte(1), rcv.getDataByte(2));

	return setAndReturnLastResult(XRV_OK);
}

/*!	\brief Requests the hardware revision from the connected device.
	\note Actually it requests master settings, but gets only hardware revision.
	\returns The hardware revision
*/
XsResultValue SerialCommunicator::getHardwareRevision()
{
	XsMessage snd(XMID_ReqMasterSettings, 1);
	snd.setBusId(XS_BID_MASTER);
	snd.setDataByte(0, 0);

	XsMessage rcv;
	if (!doTransaction(snd, rcv))
		return setAndReturnLastResult(XRV_COULDNOTREADSETTINGS);

	m_hardwareRevision = XsVersion(rcv.getDataShort(24) >> 8, rcv.getDataShort(24) & 0xff);
	return setAndReturnLastResult(XRV_OK);
}

/*!	\returns the firmware revision which was found during the OPS_InitDevice stage
*/
XsVersion SerialCommunicator::firmwareRevision()
{
	return m_firmwareRevision;
}

/*!	\returns the hardware revision only for Awinda Dongle.
*/
XsVersion SerialCommunicator::hardwareRevision()
{
	return m_hardwareRevision;
}
