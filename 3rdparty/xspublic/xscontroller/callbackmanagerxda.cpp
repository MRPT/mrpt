
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

#include "callbackmanagerxda.h"
#include <xscommon/xsens_mutex.h>

using namespace xsens;

/*! \brief Linked list item that contains a registered XsCallback handler for CallbackManagerXda
*/
struct CallbackHandlerXdaItem {
	XsCallbackPlainC* m_handler;	//!< The callback handler
	CallbackHandlerXdaItem* m_next;	//!< The next item in the list or NULL if this is the last item
};

/*! \brief Linked list item that contains a chained CallbackManagerXda
*/
struct CallbackManagerItem {
	CallbackManagerXda* m_manager;		//!< The callback managger
	CallbackManagerItem* m_next;	//!< The next item in the list or NULL if this is the last item
};

/*! \class CallbackManagerXda
	\brief Class that delegates callbacks to registered XsCallbackHandlerItems.

	CallbackManagerXda is itself an XsCallback implementer. When a callback is triggered it walks
	through its linked list of registered callbacks and calls the appropriate function in each one.

	Adding and removing handlers is done through the addXsCallbackHandlerItem() and
	removeXsCallbackHandlerItem() functions.

	The callback handler can contain so called chained managers. A chained manager receives any callback
	handler add/remove request that is done to the parent. The chained managers do not automatically
	execute all callbacks that the parent manager executes. So chaining only affects callback
	administration, not callback execution.
*/

/*! \brief Constructor, initializes the callback list.
*/
CallbackManagerXda::CallbackManagerXda()
{
	m_callbackMutex = new MutexReadWrite;
	m_handlerList = NULL;
	m_managerList = NULL;
}

/*! \brief Destructor, clears the callback list.
*/
CallbackManagerXda::~CallbackManagerXda()
{
	try {
		clearChainedManagers();
		clearCallbackHandlers(false);
		delete m_callbackMutex;
	} catch(...)
	{}
}

/*! \brief Clear the callback list
	\param chain If set to true clears all callback handlers
	\note The name is chosen like this since it is inherited and exposed by other objects
*/
void CallbackManagerXda::clearCallbackHandlers(bool chain)
{
	LockReadWrite locky(m_callbackMutex, LS_Write);
	CallbackHandlerXdaItem* currentHdlr = m_handlerList;
	while (currentHdlr)
	{
		CallbackHandlerXdaItem* next = currentHdlr->m_next;
		delete currentHdlr;
		currentHdlr = next;
	}
	m_handlerList = NULL;

	if (chain)
	{
		CallbackManagerItem* currentMgr = m_managerList;
		while (currentMgr)
		{
			currentMgr->m_manager->clearCallbackHandlers(true);
			currentMgr = currentMgr->m_next;
		}
	}
}

/*! \brief Add a handler to the list
	\param cb The handler to add to the list.
	\param chain When set to true (default) the callback is added to chained managers as well
	\note NULL and duplicate handlers are ignored, but chaining is still done.
	\note The name is chosen like this since it is inherited and exposed by other objects
*/
void CallbackManagerXda::addCallbackHandler(XsCallbackPlainC* cb, bool chain)
{
	if (!cb)
		return;

	LockReadWrite locky(m_callbackMutex, LS_Write);

	if (chain)
	{
		CallbackManagerItem* current = m_managerList;
		while (current)
		{
			current->m_manager->addCallbackHandler(cb, true);
			current = current->m_next;
		}
	}

	if (!m_handlerList)
	{
		CallbackHandlerXdaItem* current = new CallbackHandlerXdaItem;
		current->m_handler = cb;
		current->m_next = NULL;
		m_handlerList = current;
		return;
	}

	CallbackHandlerXdaItem* last = NULL;
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler == cb)
			return;
		last = current;
		current = current->m_next;
	}

	current = new CallbackHandlerXdaItem;
	current->m_handler = cb;
	current->m_next = NULL;
	last->m_next = current;
}

/*! \brief Remove a handler from the list
	\param cb The handler to remove from the list.
	\param chain When set to true (default) the callback is added to chained managers as well
	\note If \a cb is not found in the list or if \a cb is NULL, the list is not changed, but
			chaining is still done.
	\note The name is chosen like this since it is inherited and exposed by other objects
*/
void CallbackManagerXda::removeCallbackHandler(XsCallbackPlainC* cb, bool chain)
{
	if (!cb)
		return;

	LockReadWrite locky(m_callbackMutex, LS_Write);

	if (chain)
	{
		CallbackManagerItem* current = m_managerList;
		while (current)
		{
			current->m_manager->removeCallbackHandler(cb, true);
			current = current->m_next;
		}
	}

	CallbackHandlerXdaItem* last = NULL;
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler == cb)
		{
			if (last)
				last->m_next = current->m_next;
			else
				m_handlerList = current->m_next;

			delete current;
			return;
		}
		last = current;
		current = current->m_next;
	}
}

/*! \brief Clear the chained manager list
*/
void CallbackManagerXda::clearChainedManagers()
{
	LockReadWrite locky(m_callbackMutex, LS_Write);
	CallbackManagerItem* current = m_managerList;
	while (current)
	{
		CallbackManagerItem* next = current->m_next;
		delete current;
		current = next;
	}
	m_managerList = NULL;
}

/*! \brief Add a chained manager to the list
	\param cm The manager to add to the list.
	\note NULL and duplicate managers are ignored.
*/
void CallbackManagerXda::addChainedManager(CallbackManagerXda* cm)
{
	if (!cm || cm == this)
		return;

	LockReadWrite locky(m_callbackMutex, LS_Write);

	if (!m_managerList)
	{
		CallbackManagerItem* current = new CallbackManagerItem;
		current->m_manager = cm;
		current->m_next = NULL;
		m_managerList = current;
		return;
	}

	CallbackManagerItem* last = NULL;
	CallbackManagerItem* current = m_managerList;
	while (current)
	{
		if (current->m_manager == cm)
			return;
		last = current;
		current = current->m_next;
	}

	current = new CallbackManagerItem;
	current->m_manager = cm;
	current->m_next = NULL;
	last->m_next = current;
}

/*! \brief Remove achained  manager from the list
	\param cm The manager to remove from the list.
	\note If \a cm is not found in the list or if \a cm is NULL, the list is not changed.
*/
void CallbackManagerXda::removeChainedManager(CallbackManagerXda* cm)
{
	if (!cm)
		return;

	LockReadWrite locky(m_callbackMutex, LS_Write);

	CallbackManagerItem* last = NULL;
	CallbackManagerItem* current = m_managerList;
	while (current)
	{
		if (current->m_manager == cm)
		{
			if (last)
				last->m_next = current->m_next;
			else
				m_managerList = current->m_next;

			delete current;
			return;
		}
		last = current;
		current = current->m_next;
	}
}

/*! \brief Copy all handlers from this manager into \a cm
	\param cm The CallbackManagerXda to copy the handlers to
	\param chain Whether to propagate the added handlers to chained managers
*/
void CallbackManagerXda::copyCallbackHandlersTo(CallbackManagerXda* cm, bool chain)
{
	cm->copyCallbackHandlersFrom(this, chain);
}

/*! \brief Copy all handlers from \a cm into this manager
	\param cm The CallbackManagerXda to copy the handlers from
	\param chain Whether to propagate the added handlers to chained managers
*/
void CallbackManagerXda::copyCallbackHandlersFrom(CallbackManagerXda* cm, bool chain)
{
	LockReadWrite locky(m_callbackMutex, LS_Write);
	LockReadWrite locky2(cm->m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = cm->m_handlerList;
	while (current)
	{
		addCallbackHandler(current->m_handler, chain);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onDeviceStateChanged() callback forwarding function
void CallbackManagerXda::onDeviceStateChanged(XsDevice* dev, XsDeviceState newState, XsDeviceState oldState)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onDeviceStateChanged)
			current->m_handler->m_onDeviceStateChanged(current->m_handler, dev, newState, oldState);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onLiveDataAvailable() callback forwarding function
void CallbackManagerXda::onLiveDataAvailable(XsDevice* dev, const XsDataPacket* packet)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onLiveDataAvailable)
			current->m_handler->m_onLiveDataAvailable(current->m_handler, dev, packet);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onAllLiveDataAvailable() callback forwarding function
void CallbackManagerXda::onAllLiveDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onAllLiveDataAvailable)
			current->m_handler->m_onAllLiveDataAvailable(current->m_handler, devs, packets);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onMissedPackets() callback forwarding function
void CallbackManagerXda::onMissedPackets(XsDevice* dev, int count, int first, int last)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onMissedPackets)
			current->m_handler->m_onMissedPackets(current->m_handler, dev, count, first, last);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onDataUnavailable() callback forwarding function
void CallbackManagerXda::onDataUnavailable(XsDevice* dev, int64_t packetId)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onDataUnavailable)
			current->m_handler->m_onDataUnavailable(current->m_handler, dev, packetId);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onWakeupReceived() callback forwarding function
void CallbackManagerXda::onWakeupReceived(XsDevice* dev)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onWakeupReceived)
			current->m_handler->m_onWakeupReceived(current->m_handler, dev);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onProgressUpdated() callback forwarding function
void CallbackManagerXda::onProgressUpdated(XsDevice* dev, int curr, int total, const XsString* identifier)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onProgressUpdated)
			current->m_handler->m_onProgressUpdated(current->m_handler, dev, curr, total, identifier);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onWriteMessageToLogFile() callback forwarding function
int CallbackManagerXda::onWriteMessageToLogFile(XsDevice* dev, const XsMessage* message)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	bool rv = true;
	while (current)
	{
		if (current->m_handler->m_onWriteMessageToLogFile)
			rv = current->m_handler->m_onWriteMessageToLogFile(current->m_handler, dev, message) && rv;
		current = current->m_next;
	}
	return rv?1:0;
}

//! \brief The XsCallback::onBufferedDataAvailable() callback forwarding function
void CallbackManagerXda::onBufferedDataAvailable(XsDevice* dev, const XsDataPacket* data)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onBufferedDataAvailable)
			current->m_handler->m_onBufferedDataAvailable(current->m_handler, dev, data);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onAllBufferedDataAvailable() callback forwarding function
void CallbackManagerXda::onAllBufferedDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onAllBufferedDataAvailable)
			current->m_handler->m_onAllBufferedDataAvailable(current->m_handler, devs, packets);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onConnectivityChanged() callback forwarding function
void CallbackManagerXda::onConnectivityChanged(XsDevice* dev, XsConnectivityState newState)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onConnectivityChanged)
			current->m_handler->m_onConnectivityChanged(current->m_handler, dev, newState);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onInfoResponse() callback forwarding function
void CallbackManagerXda::onInfoResponse(XsDevice* dev, XsInfoRequest request)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onInfoResponse)
			current->m_handler->m_onInfoResponse(current->m_handler, dev, request);
		current = current->m_next;
	}
}

//! \brief The Xscallback::onError() callback forwarding function
void CallbackManagerXda::onError(XsDevice* dev, XsResultValue error)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onError)
			current->m_handler->m_onError(current->m_handler, dev, error);
		current = current->m_next;
	}
}

//! \brief The Xscallback::onNonDataMessage() callback forwarding function
void CallbackManagerXda::onNonDataMessage(XsDevice* dev, XsMessage const * message)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onNonDataMessage)
			current->m_handler->m_onNonDataMessage(current->m_handler, dev, message);
		current = current->m_next;
	}
}

//! \brief The Xscallback::onMessageReceivedFromDevice() callback forwarding function
void CallbackManagerXda::onMessageDetected(XsDevice* dev, XsProtocolType type, XsByteArray const * rawMessage)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onMessageDetected)
			current->m_handler->m_onMessageDetected(current->m_handler, dev, type, rawMessage);
		current = current->m_next;
	}
}

//! \brief The Xscallback::onMessageReceivedFromDevice() callback forwarding function
void CallbackManagerXda::onMessageReceivedFromDevice(XsDevice* dev, XsMessage const * message)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onMessageReceivedFromDevice)
			current->m_handler->m_onMessageReceivedFromDevice(current->m_handler, dev, message);
		current = current->m_next;
	}
}

//! \brief The Xscallback::onMessageSentToDevice() callback forwarding function
void CallbackManagerXda::onMessageSentToDevice(XsDevice* dev, XsMessage const * message)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onMessageSentToDevice)
			current->m_handler->m_onMessageSentToDevice(current->m_handler, dev, message);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onDataAvailable() callback forwarding function
void CallbackManagerXda::onDataAvailable(XsDevice* dev, const XsDataPacket* packet)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onDataAvailable)
			current->m_handler->m_onDataAvailable(current->m_handler, dev, packet);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onAllDataAvailable() callback forwarding function
void CallbackManagerXda::onAllDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onAllDataAvailable)
			current->m_handler->m_onAllDataAvailable(current->m_handler, devs, packets);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onRecordedDataAvailable() callback forwarding function
void CallbackManagerXda::onRecordedDataAvailable(XsDevice* dev, const XsDataPacket* packet)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onRecordedDataAvailable)
			current->m_handler->m_onRecordedDataAvailable(current->m_handler, dev, packet);
		current = current->m_next;
	}
}

//! \brief The XsCallback::onAllRecordedDataAvailable() callback forwarding function
void CallbackManagerXda::onAllRecordedDataAvailable(XsDevicePtrArray* devs, const XsDataPacketPtrArray* packets)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onAllRecordedDataAvailable)
			current->m_handler->m_onAllRecordedDataAvailable(current->m_handler, devs, packets);
		current = current->m_next;
	}
}

void CallbackManagerXda::onTransmissionRequest(int channelId, const XsByteArray* data)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onTransmissionRequest)
			current->m_handler->m_onTransmissionRequest(current->m_handler, channelId, data);
		current = current->m_next;
	}
}

//! \brief The Xscallback::onRestoreCommunication callback forwarding function
void CallbackManagerXda::onRestoreCommunication(const XsString* portName, XsResultValue result)
{
	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		if (current->m_handler->m_onRestoreCommunication)
			current->m_handler->m_onRestoreCommunication(current->m_handler, portName, result);
		current = current->m_next;
	}
}

/*! \returns True if at least one callback handler defines the given \a function
	\param functionOffset The offset of one of XsCallbackPlainC's function pointers, ie offsetof(XsCallbackPlainC, m_onDeviceStateChanged)
	\note This is an experimental function, it may still be removed at any time if it doesn't work properly
*/
bool CallbackManagerXda::haveCallback(size_t functionOffset) const
{
	if (functionOffset > (sizeof(XsCallbackPlainC) - sizeof(void*)))
		return false;	// we don't have this callback if it's beyond the end

	LockReadWrite locky(m_callbackMutex, LS_Read);
	CallbackHandlerXdaItem* current = m_handlerList;
	while (current)
	{
		char* plain = (char*) current->m_handler;
		if (((void*) (plain + functionOffset)) != nullptr)
			return true;
		current = current->m_next;
	}
	return false;
}
