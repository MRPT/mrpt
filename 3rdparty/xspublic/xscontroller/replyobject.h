
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

#ifndef REPLYOBJECT_H
#define REPLYOBJECT_H

#include <xstypes/xsmessage.h>
#include <xstypes/xsdeviceid.h>

namespace xsens
{
class ReplyMonitor;
class Mutex;
class WaitCondition;
}

/*! \class ReplyObject
	\brief Abstract reply object. Blocks on a semaphore when requesting the message until the message has been set by the reply monitor.
*/
class ReplyObject
{
public:
	explicit ReplyObject();
	virtual ~ReplyObject();

	void setMessage(const XsMessage& msg);
	XsMessage message(uint32_t timeout);

	/*! \returns True when a message is a valid reply message for this reply object
		\param[in] message The message to check
	*/
	virtual bool isReplyFor(XsMessage const & message) = 0;

	//! \returns The message ID that this reply object is waiting for
	virtual uint8_t msgId() const = 0;

private:
	xsens::Mutex* m_mutex;
	xsens::WaitCondition *m_waitCondition;
	XsMessage m_message;
	bool m_delivered;
};

/*! \class MidReplyObject
	\brief Reply object that only checks the message identifier.
*/
class MidReplyObject : public ReplyObject
{
public:
	MidReplyObject(uint8_t messageId);
	~MidReplyObject(void);

	virtual bool isReplyFor(XsMessage const & message);
	virtual uint8_t msgId() const;
private:
	uint8_t m_messageId;
};

/*! \class MidAndDataReplyObject
	\brief Reply object that checks the message identifier and data in the data field
*/
class MidAndDataReplyObject : public ReplyObject
{
public:
	MidAndDataReplyObject(uint8_t messageId, XsSize offset, XsSize size, uint8_t const * data);
	~MidAndDataReplyObject();
	void setData(uint8_t const * data);
	virtual bool isReplyFor(XsMessage const & message);
	virtual uint8_t msgId() const;
private:
	void freeData();
	uint8_t m_messageId;
	XsSize m_dataOffset;
	XsSize m_dataSize;
	uint8_t * m_data;
};

#endif

