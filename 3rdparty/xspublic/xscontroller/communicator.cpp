
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

#include "communicator.h"
#include "protocolhandler.h"
#include "replymonitor.h"
#include <xstypes/xsportinfo.h>
#include "usbinterface.h"
#include <xstypes/xsdid.h>
#include <xstypes/xsbusid.h>
#include "xsdeviceconfiguration.h"
#include "xsusbhubinfo.h"
#include "xsscanner.h"
#include <xscommon/xsens_debugtools.h>
#include <xscommon/xsens_janitors.h>
#include "xsdevice_def.h"

using namespace xsens;
using namespace XsTime;

/*! \brief Constructor, creates some management objects and clears the rest by calling initialize()
*/
Communicator::Communicator(void)
	: m_preparedForDestruction(false)
	, m_masterInfo(nullptr)
	, m_protocolManager(new ProtocolManager(*this))
	, m_masterDeviceId(0)
	, m_replyMonitor(new ReplyMonitor)
	, m_lastResult(XRV_OK)
	, m_defaultTimeout(500)
{
	ProtocolManager::value_type addOk = protocolManager()->add(new ProtocolHandler());
	assert(!addOk.isNull());

	JLDEBUGG("Created " << (void*) this);
}

/*! \brief Destructor, waits for the last scheduled task to complete and then cleans up the object by calling clear()
*/
Communicator::~Communicator()
{
	JLDEBUGG("Destroyed " << (void*) this);
	assert(m_preparedForDestruction);
}

/*! \brief Prepares communicator for destruction
*/
void Communicator::prepareForDestruction()
{
	m_replyMonitor.reset(nullptr);
	m_protocolManager.reset();

	m_preparedForDestruction = true;
}


/*! \brief Sets a master device.
	\param masterDevice a master device.
*/
void Communicator::setMasterDevice(XsDevice *masterDevice)
{
	JLDEBUGG(masterDevice);
	assert(m_masterInfo == nullptr);
	m_masterInfo = masterDevice;
}

/*!	\returns a master device
*/
XsDevice* Communicator::masterDevice() const
{
	return m_masterInfo;
}

/*!	\returns a protocol manager
*/
std::shared_ptr<ProtocolManager> Communicator::protocolManager() const
{
	return m_protocolManager;
}

/*!	\brief Sets a master device ID.
	\param id device ID.
*/
void Communicator::setMasterDeviceId(const XsDeviceId &id)
{
	m_masterDeviceId = id;
}

/*! \returns a master device ID
*/
XsDeviceId Communicator::masterDeviceId() const
{
	return m_masterDeviceId;
}

/*!	\brief Get the result value of the last operation.
	\details The result values are codes that describe a failure in more detail.
	\returns the last known error code
	\sa resultText(XsResultValue), lastResultText()
*/
XsResultValue Communicator::lastResult() const
{
	return m_lastResult;
}

/*!	\brief Get the accompanying error text for the value returned by lastResult()
	It may provide situation-specific information instead.
	\returns a human readable error description
	\sa resultText(XsResultValue), lastResult()
*/
XsString Communicator::lastResultText() const
{
	return m_lastResultText;
}

/*! \returns a child device count
*/
XsSize Communicator::childDeviceCount() const
{
	return m_masterInfo ? m_masterInfo->deviceConfigurationConst().numberOfDevices() : 0;
}

/*! \brief Handles a \a message
*/
void Communicator::handleMessage(const XsMessage &message)
{
	// handle one message at a time. This is only really necessary for dual-stream interfaces such as to the bodypack
	xsens::Lock locky(&m_handleMux);

	JLTRACEG("Received " << JLHEXLOG(message.getMessageId()) << " size " << message.getTotalMessageSize());
	if (message.getMessageId() == XMID_Error)
	{
		char buffer[256];
		XsSize sz = message.getTotalMessageSize();
		const uint8_t* m = message.getMessageStart();
		for (XsSize i = 0; i < sz; ++i)
			sprintf(buffer+2*i, "%02X", m[i]);
		buffer[2*sz] = 0;
		JLALERTG("Error message received: " << buffer);
		JLIF(gJournal, JLL_Alert, m_replyMonitor->dumpObjectList(gJournal, JLL_Alert));
	}
	if (!m_replyMonitor->addReply(message) && m_masterInfo)
		m_masterInfo->handleMessage(message);
}

/*! \brief Write a message and await the reply
*/
bool Communicator::doTransaction(const XsMessage &msg, uint32_t timeout)
{
	XsMessage rcv;
	return doTransaction(msg, rcv, timeout);
}

/*! \brief Write a message and await the reply
*/
bool Communicator::doTransaction(const XsMessage &msg)
{
	return doTransaction(msg, defaultTimeout());
}

/*! \brief Write a message and await the reply
*/
bool Communicator::doTransaction(const XsMessage &msg, XsMessage &rcv)
{
	return doTransaction(msg, rcv, defaultTimeout());
}

/*! \brief Sets the last result
	\param res a result value
	\param text a text string
*/
void Communicator::setLastResult(XsResultValue res, XsString const& text) const
{
	(void)setAndReturnLastResult(res, text);
}

/*! \brief Sets the last result and returns it
	\param res a result value
	\param text a text string
	\returns XRV_OK if successful
*/
XsResultValue Communicator::setAndReturnLastResult(XsResultValue res, XsString const& text) const
{
	m_lastResultText = text;
	return m_lastResult = res;
}

/*! \brief Do a sanity check on a potential message
	\param msg a message
	\details This check is done to prevent message loss due to a 'valid' message in a message
	Check if the busId can be valid. It can be XS_BID_MASTER and is must be below the number of devices
	The XS_BID_BROADCAST cannot be checked due to a bug in AwindaStation firmware 1.0.9 which sends flushing indications with BID 0
	Later more checks like odd message ID checks may be added here
	\returns true if check is successful
*/
bool Communicator::sanityCheck(XsMessage const & msg) const
{
	if (m_masterInfo)
		return m_masterInfo->messageLooksSane(msg);

	return true;
}

/*! \brief Add a MidReplyObject
	\param[in] mid the id of the message to wait for
	\returns a shared pointer to a reply object
*/
std::shared_ptr<ReplyObject> Communicator::addReplyObject(uint8_t mid)
{
	assert(m_replyMonitor != nullptr);
	return m_replyMonitor->addReplyObject(new MidReplyObject(mid));
}

/*! \brief Add a MidAndDataReplyObject
	\param[in] mid the message id of the message to wait for
	\param[in] offset the offset in the data part of the message
	\param[in] size the size of the data in the data part of the message
	\param[in] data pointer to data to wait for (this object does not take ownership of the data)
	\returns a shared pointer to a reply object
*/
std::shared_ptr<ReplyObject> Communicator::addReplyObject(uint8_t mid, XsSize offset, XsSize size, uint8_t const * data)
{
	assert(m_replyMonitor != nullptr);
	return m_replyMonitor->addReplyObject(new MidAndDataReplyObject(mid, offset, size, data));
}

/*! \brief Add a custom ReplyObject
	\param[in] obj The reply object to add
	\returns a shared pointer to the supplied reply object
*/
std::shared_ptr<ReplyObject> Communicator::addReplyObject(ReplyObject* obj)
{
	assert(m_replyMonitor != nullptr && obj != nullptr);
	return m_replyMonitor->addReplyObject(obj);
}

void Communicator::addProtocolHandler(IProtocolHandler* handler)
{
	assert(handler != 0);
	ProtocolManager::value_type addOk = protocolManager()->add(handler);
	assert(!addOk.isNull());
}

/*! \brief Removes a protocol handler
	\param type a \ref XsProtocolType
*/
void Communicator::removeProtocolHandler(XsProtocolType type)
{
	bool removeOk = protocolManager()->remove(type);
	assert(removeOk);
	(void)removeOk; // Prevent unused variable warning in release builds
}

/*! \returns true when a protocol with type \a type has been added
	\param[in] type The protocol type to check
*/
bool Communicator::hasProtocol(XsProtocolType type) const
{
	return protocolManager()->hasProtocol(type);
}

/*! \brief Set the credentials required for using the device
	\details Mostly used for network access
	\param id The username or system ID or similar
	\param key The authentication key or password or similar
*/
void Communicator::setCredentials(XsString const& id, XsString const& key)
{
	(void) id;
	(void) key;
}

/*! \brief Destroys the communicator. */
void Communicator::destroy()
{
	prepareForDestruction();
	delete this;
}

bool Communicator::isLoadLogFileInProgress() const
{
	return false;
}

bool Communicator::allowReprocessing() const
{
	return true;
}
