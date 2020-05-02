
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

#include "protocolmanager.h"
#include "iprotocolhandler.h"
#include "communicator.h"
#include <algorithm>

// Remove windows global min/max macros
#undef min
#undef max

/*! \class ProtocolManager
	\brief Supplies multi-protocol data stream parsing
	\details This class provides functionality for dealing with multiple or changing communication
	protocols in a data stream. The XCommunicator class uses this class to parse a raw byte stream
	into XsMessage objects.
*/

/*! \brief Default constructor
*/
ProtocolManager::ProtocolManager(Communicator const & communicator)
: m_communicator(communicator)
{
}

/*! \brief Default destructor
*/
ProtocolManager::~ProtocolManager() throw()
{
}

/*! \returns A const interator of the protocol handlers list beginning
*/
ProtocolManager::const_iterator ProtocolManager::begin() const
{
	return m_protocolHandlers.begin();
}

/*! \returns A const interator of the protocol handlers list end
*/
ProtocolManager::const_iterator ProtocolManager::end() const
{
	return m_protocolHandlers.end();
}

/*! \brief Searches for a raw message in a raw data
	\param[out] type The protocol type that was used.
	\param[in] raw The byte array to search in.
	\returns The found message location
*/
MessageLocation ProtocolManager::findMessage(XsProtocolType& type, const XsByteArray& raw)
{
	MessageLocation bestMessageLocation;
	XsProtocolType bestProtocolType = XPT_Xbus;
	container_type::iterator bestHandlerIter = m_protocolHandlers.end();

	for (container_type::iterator i = m_protocolHandlers.begin(); i != m_protocolHandlers.end(); ++i)
	{
		assert((*i).operator->() != 0);
		IProtocolHandler const & handler = **i;
		XsProtocolType currentProtocolType = static_cast<XsProtocolType>(handler.type());
		MessageLocation currentMessageLocation = handler.findMessage(type, raw);

		if (currentMessageLocation.isValid())
		{
			// Message is valid
			if (!bestMessageLocation.isValid() || (currentMessageLocation.m_startPos < bestMessageLocation.m_startPos))
			{
				// Message is a better match
				bestMessageLocation = currentMessageLocation;
				bestProtocolType = currentProtocolType;
				bestHandlerIter = i;
			}

			// Stop searching if the location is as good as it gets
			if (bestMessageLocation.m_startPos == 0)
			{
				break;
			}
		}
		else
		{
			// No valid location/message produced. Continue searching.
			// location may still contain useful information
			if ((currentMessageLocation.m_startPos >= 0 && currentMessageLocation.m_size < 0 &&
				(!bestMessageLocation.isValid() || bestMessageLocation.m_startPos > currentMessageLocation.m_startPos)) ||
				(currentMessageLocation.m_incompletePos >= 0 && currentMessageLocation.m_incompleteSize > 0 &&
				(!bestMessageLocation.isValid() || bestMessageLocation.m_startPos > currentMessageLocation.m_incompletePos)))
			{
				bestMessageLocation = currentMessageLocation;
				bestProtocolType = currentProtocolType;
			}
				
		}
	}

	// Move the best handler to the front of the list to speed up future searches
	if (bestHandlerIter != m_protocolHandlers.end() && bestHandlerIter != m_protocolHandlers.begin())
	{
		value_type bestHandler = *bestHandlerIter;
		container_type& cont = m_protocolHandlers;
		cont.erase(bestHandlerIter);
		cont.push_front(bestHandler);
	}
	type = bestProtocolType;
	return bestMessageLocation;
}

/*! \brief Converts \a raw data using \a location into a %XsMessage object.
	\param[in] type The protocol type to use.
	\param[out] location The location of a message to convert from \a raw data.
	\param[in] raw The raw byte stream.
	\returns A %XsMessage object that was converted from raw byte stream.
*/
XsMessage ProtocolManager::convertToMessage(XsProtocolType& type, MessageLocation& location, const XsByteArray& raw)
{
	for (auto const& handler : m_protocolHandlers)
		if (handler->type() == type)
			return handler->convertToMessage(location, raw);

	return XsMessage();
}

/*! \brief Removes a protocol handler of a specified type
	\param[in] type The type of the protocol handler to remove
	\returns True if successful
*/
bool ProtocolManager::remove(XsProtocolType type)
{
	bool result = false;
	container_type::iterator i = m_protocolHandlers.begin();
	while (i != m_protocolHandlers.end()) {
		if ((*i)->type() == type) {
			// Increment the incrementing iterator before removing the pointed to element to
			// prevent icrementing an invalidated iterator
			container_type::iterator toErase = i;
			++i;
			m_protocolHandlers.erase(toErase);
			result = true;
		}
		else {
			++i;
		}
	}
	return result;
}


/*! \returns true when a protocol with type \a type has been added
	\param[in] type The protocol type to check
*/
bool ProtocolManager::hasProtocol(XsProtocolType type) const
{
	bool result = false;
	for (container_type::const_iterator i = m_protocolHandlers.begin(); i != m_protocolHandlers.end(); ++i) {
		if ((*i)->type() == type) {
			result = true;
		}
	}
	return result;
}


ProtocolManager::value_type ProtocolManager::add(IProtocolHandler* handler)
{
	assert(handler != 0);
	// check for duplicates first
	for (container_type::iterator i = m_protocolHandlers.begin(); i != m_protocolHandlers.end(); ++i)
		if (handler->type() == (*i)->type())
			return (*i);

	m_protocolHandlers.push_back(value_type(handler));
	value_type inserted = m_protocolHandlers.back();
	return inserted;
}

/*! \brief Clears the protocol handlers list
*/
void ProtocolManager::clear()
{
	m_protocolHandlers.clear();
}

/*! \brief Check a message for a validity
	\param[in] message The message to check
	\returns True if valid
*/
bool ProtocolManager::validateMessage(XsMessage const & message) const
{
	return m_communicator.sanityCheck(message);
}

/*! \returns the minimum message size of the last successful protocol handler
*/
int ProtocolManager::likelyMinimumMessageSize() const
{
	if (m_protocolHandlers.empty())
		return 0;
	return (*m_protocolHandlers.begin())->minimumMessageSize();
}

