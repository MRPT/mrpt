
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

#include "replyobject.h"
#include "replymonitor.h"
#include <xstypes/xsthread.h>
#include <xstypes/xsresultvalue.h>

using namespace xsens;

/*! \brief Default constructor */
ReplyObject::ReplyObject()
	: m_mutex(new Mutex)
	, m_waitCondition(new WaitCondition(*m_mutex))
	, m_delivered(false)
{
}

/*! \brief Default destructor */
ReplyObject::~ReplyObject()
{
	try
	{
		delete m_waitCondition;
		delete m_mutex;
	}
	catch(...)
	{}
}

/*! \brief Sets a message as reply message and trigger the semaphore which will unblock any waiting message() calls
*/
void ReplyObject::setMessage(const XsMessage& msg)
{
	xsens::Lock locker(m_mutex);

	m_message = msg;
	m_delivered = true;
	m_waitCondition->signal();
}

/*! \brief Blocks until a message has been set by setMessage() then returns that message
*/
XsMessage ReplyObject::message(uint32_t timeout)
{
	xsens::Lock locker(m_mutex);

	if (!m_delivered)
		m_waitCondition->wait(timeout);
	if (m_delivered)
		return m_message;
	else
		return XsMessage();
}

/*! \brief MidReplyObject constructor
	\param[in] messageId the id of the message to wait for
*/
MidReplyObject::MidReplyObject(uint8_t messageId)
	: ReplyObject()
	, m_messageId(messageId)
{
}

/*! \brief MidReplyObject destructor
*/
MidReplyObject::~MidReplyObject()
{
}

/*! \returns the message ID that this reply object is waiting for
*/
uint8_t MidReplyObject::msgId() const
{
	return m_messageId;
}

/*! \returns True when a message is a valid reply message for this reply object
	\param[in] msg the message to check
*/
bool MidReplyObject::isReplyFor(XsMessage const & msg)
{
	if (m_messageId == msg.getMessageId())
		return true;

	if (msg.getMessageId() == XMID_Error)
	{
		assert(msg.getDataSize());
		return (static_cast<XsResultValue>(msg.getDataByte()) != XRV_DATAOVERFLOW);
	}
	return false;
}

/*! \brief MidAndDataReplyObject constructor
	\param[in] messageId the message id of the message to wait for
	\param[in] offset the offset in the data part of the message
	\param[in] size the size of the data in the data part of the message
	\param[in] data pointer to data to wait for (this object does not take ownership of the data)
*/
MidAndDataReplyObject::MidAndDataReplyObject(uint8_t messageId, XsSize offset, XsSize size, uint8_t const * data)
	: ReplyObject()
	, m_messageId(messageId)
	, m_dataOffset(offset)
	, m_dataSize(size)
	, m_data(0)
{
	assert(m_dataSize > 0);
	setData(data);
}

/*! \brief MidAndDataReplyObject destructor
*/
MidAndDataReplyObject::~MidAndDataReplyObject()
{
	try
	{
		freeData();
	}
	catch (...)
	{
		assert(false);
	}
}

/*! \returns the message ID that this reply object is waiting for
*/
uint8_t MidAndDataReplyObject::msgId() const
{
	return m_messageId;
}

/*! \returns true when a message is a valid reply message for this reply object
	\param[in] msg the message to check
*/
bool MidAndDataReplyObject::isReplyFor(XsMessage const & msg)
{
	if (msg.getMessageId() == XMID_Error)
		return true;
	if (m_messageId != msg.getMessageId())
		return false;
	return memcmp(msg.getDataBuffer(m_dataOffset), m_data, m_dataSize) == 0;
}

/*! \brief Frees allocated data (if any)
*/
void MidAndDataReplyObject::freeData()
{
	if (m_data != 0)
	{
		free(m_data);
		m_data = 0;
	}
}

/*! \brief Copies data from 'data' into this object. This is the data to wait for.
	\param[in] data the data to copy into this object and wait for
*/
void MidAndDataReplyObject::setData(uint8_t const * data)
{
	freeData();
	if (data != 0)
	{
		m_data = reinterpret_cast<uint8_t*>(malloc(m_dataSize));
		if (m_data != 0)
			memcpy(m_data, data, m_dataSize);
	}
}
