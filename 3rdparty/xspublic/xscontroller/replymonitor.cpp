
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

#include "replymonitor.h"
#include "replyobject.h"
#include <algorithm>

namespace xsens {

/*! \class ReplyObjectDeleter
	\brief A class that deletes a reply object and removes it from the given monitor
*/
class ReplyObjectDeleter
{
public:
	// create a reply object deleter
	ReplyObjectDeleter(ReplyObjectRemover *monitor = NULL) :
		m_monitor(monitor)
	{
	}

	// delete the ReplyObject and remove it from m_monitor
	// not necessarily in that order
	void operator()(ReplyObject *p) const
	{
		if (m_monitor)
			m_monitor->removeObject(p);
		delete p;
	}
private:
	ReplyObjectRemover *m_monitor;
};


/*! \class ReplyMonitor
	\brief A monitor class for receiving replies messages in a thread
	\details This class monitors for a desired message, and releases a semaphore when the message is received
*/

/*! \brief Default constructor */
ReplyMonitor::ReplyMonitor(void)
{
}

/*! \brief Default destructor */
ReplyMonitor::~ReplyMonitor(void)
{
}

/*! \brief Add a reply object to the reply monitor
	\param[in] replyObject the ReplyObject to add
	\returns a shared pointer to this object
*/
std::shared_ptr<ReplyObject> ReplyMonitor::addReplyObject(ReplyObject* replyObject)
{
	xsens::Lock locky(&m_mutex);
	m_objectList.push_back(replyObject);
	return std::shared_ptr<ReplyObject>(replyObject, ReplyObjectDeleter(this));
}

/*! \brief Remove an object from the list
	\param obj The shared pointer to the object to remove
	\note This does not delete the object
*/
void ReplyMonitor::removeObject(std::shared_ptr<ReplyObject> &obj)
{
	removeObject(obj.get());
}

/*! \brief Remove an object from the list
	\param obj The object to remove
	\note This does not delete the object
*/
void ReplyMonitor::removeObject(ReplyObject *obj)
{
	xsens::Lock locky(&m_mutex);
	if (m_objectList.empty())
		return;
	std::vector<ReplyObject *>::iterator it = std::find(m_objectList.begin(), m_objectList.end(), obj);
	if (it == m_objectList.end())
		return;

	m_objectList.erase(it);
}

/*! \brief Put a reply in the monitor
	\param message The message to add as a reply
	\returns true if the message is delivered
*/
bool ReplyMonitor::addReply(const XsMessage& message)
{
	xsens::Lock locky(&m_mutex);
	size_t numElements = m_objectList.size();
	for (size_t i = 0; i < numElements; i++)
	{
		if (m_objectList[i]->isReplyFor(message))
		{
			ReplyObject* tmp = m_objectList[i];
			m_objectList.erase(m_objectList.begin()+i);
			tmp->setMessage(message);
			return true;
		}
	}
	return false;
}

/*! \brief Dumps the current list of objects to wait for to the supplied journaller
*/
void ReplyMonitor::dumpObjectList(Journaller* journal, JournalLogLevel level) const
{
	xsens::Lock locky(&m_mutex);
	size_t numElements = m_objectList.size();
	JLGENERIC(journal, level, "Waiting for " << numElements << " objects");
	for (size_t i = 0; i < numElements; i++)
	{
		JLGENERIC(journal, level, i << ": msg ID = " << JLHEXLOG((int) m_objectList[i]->msgId()));
	}
}

}	// namespace xsens
