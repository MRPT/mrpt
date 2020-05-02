
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

#include "messageextractor.h"
#include "xscontrollerconfig.h"
#include <xstypes/xsmessagearray.h>

/*! \class MessageExtractor

	Helper class that extracts XsMessages from a stream of data. The user must call the \a processNewData function every time a new block of data is available.
	It is advised not to process too small blocks of data (e.g. per byte); Every single message must not span more than \a m_maxIncompleteRetryCount blocks
	to guarantee correct operation.

	 A MessageExtractor object maintains a buffer representing a sliding window over the data stream that is just big enough to contain any incompletely received
	 XsMessage. The user can explicitly clear this buffer using the \a clearBuffer function
*/


/*! \brief Constructor
	\param protocolManager: the protocol manager to use for finding messages in the buffered data
*/
MessageExtractor::MessageExtractor(std::shared_ptr<IProtocolManager> protocolManager)
	: m_protocolManager(protocolManager)
	, m_retryTimeout(0)
	, m_buffer()
	, m_maxIncompleteRetryCount(5)
{
}

/*! \brief Processes new incoming data for message extraction

	\param devicePtr: %XsDevice pointer to call a onMessageDetected callback
	\param newData: Buffer that contains the newly arrived data
	\param messages: Newly extracted messages are stored in this vector. This vector will be cleared upon function entry
	\returns XRV_OK if one or more messages were successfully extracted. Something else if not
*/
XsResultValue MessageExtractor::processNewData(XsDevice* devicePtr, XsByteArray const& newData, std::deque<XsMessage> &messages)
{
	if (!m_protocolManager)
		return XRV_ERROR;

#ifdef XSENS_DEBUG
	XsSize prevSize = m_buffer.size();
#endif
	if (newData.size())
		m_buffer.append(newData);
#ifdef XSENS_DEBUG
	assert(m_buffer.size() == newData.size() + prevSize);
#endif

	XsSize popped = 0;
	messages.clear();

	auto retval = [&]()
	{
		if (popped > 0)
			m_buffer.pop_front(popped);
		if (messages.empty())
			return XRV_TIMEOUTNODATA;

		return XRV_OK;
	};

	while (true)
	{
		assert(popped <= m_buffer.size());

		XsByteArray raw(m_buffer.data() + popped, m_buffer.size() - popped, XSDF_None);

		XsProtocolType type;
		MessageLocation location = m_protocolManager->findMessage(type, raw);

		if (location.isValid())
		{
			XsByteArray detectedMessage(&raw[location.m_startPos], location.m_size, XSDF_None);
			if (devicePtr != nullptr)
				devicePtr->onMessageDetected(type, detectedMessage);

			XsMessage message = m_protocolManager->convertToMessage(type, location, raw);

			if (location.isValid() && !message.empty() && m_protocolManager->validateMessage(message))
			{
				assert(location.m_startPos == -1 || location.m_incompletePos == -1 || location.m_incompletePos < location.m_startPos);

				if (location.m_startPos > 0)
				{
					// We are going to skip something
					if (location.m_incompletePos != -1)
					{
						// We are going to skip an incomplete but potentially valid message
						// First wait a couple of times to see if we can complete that message before skipping
						if (m_retryTimeout++ < m_maxIncompleteRetryCount)
						{
							// wait a bit until we have more data
							// but already pop the data that we know contains nothing useful
							if (location.m_incompletePos > 0)
							{
								JLALERTG("Skipping " << location.m_incompletePos << " bytes from the input buffer");
								popped += location.m_incompletePos;
							}

							return retval();
						}
						else
						{
							// We've waited a bit for the incomplete message to complete but it never completed
							// So: We are going to skip an incomplete but potentially valid message
							JLALERTG("Skipping " << location.m_startPos << " bytes from the input buffer that may contain an incomplete message at " << location.m_incompletePos
								<< " found: " << (int)message.getTotalMessageSize()
								<< std::hex << std::setfill('0')
								<< " First bytes " << std::setw(2) << (int)message.getMessageStart()[0]
								<< " " << std::setw(2) << (int)message.getMessageStart()[1]
								<< " " << std::setw(2) << (int)message.getMessageStart()[2]
								<< " " << std::setw(2) << (int)message.getMessageStart()[3]
								<< " " << std::setw(2) << (int)message.getMessageStart()[4]
								<< std::dec << std::setfill(' '));
						}
					}
					else
					{
						// We are going to skip something but we are not going to skip an incomplete but potentially valid message
						JLALERTG("Skipping " << location.m_startPos << " bytes from the input buffer");
					}
				}
				if (m_retryTimeout)
				{
					JLTRACEG("Resetting retry count from " << m_retryTimeout);
					m_retryTimeout = 0;
				}

				// message is valid, remove data from cache
				popped += location.m_size + location.m_startPos;
				messages.push_back(message);
			}
			else
			{
				if (type == XPT_Nmea)
					popped += location.m_size + location.m_startPos;
				else
					return retval();
			}
		}
		else
		{
			return retval();
		}
	}
}

/*! \brief Clears the processing buffer
*/
void MessageExtractor::clearBuffer()
{
	JLDEBUGG(this);
	m_buffer.clear();
}

/*! \brief Sets the maximum number of process attempts before advancing over an incompletely received message.
	\param max The maximum number set
	\return Old maximum number
*/
int MessageExtractor::setMaxIncompleteRetryCount(int max)
{
	int rv = m_maxIncompleteRetryCount;
	m_maxIncompleteRetryCount = max;
	return rv;
}
