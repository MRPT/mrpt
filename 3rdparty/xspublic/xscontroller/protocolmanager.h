
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

#ifndef PROTOCOLMANAGER_H
#define PROTOCOLMANAGER_H

#include "messagelocation.h"
#include "iprotocolmanager.h"
#include <xscommon/xsens_nonintrusive_shared_pointer.h>
#include <list>
#include <memory>
#include "xsprotocoltype.h"

class IProtocolHandler;
struct Communicator;
struct XsByteArray;
struct XsMessage;


class ProtocolManager : public IProtocolManager
{
public:

	//! \brief A typedef for a value type
	typedef xsens::NonIntrusiveSharedPointer<IProtocolHandler> value_type;

	//! \brief A typedef for a container type
	typedef std::list<value_type> container_type;

	//! \brief A typedef for a const iterator
	typedef container_type::const_iterator const_iterator;

	explicit ProtocolManager(Communicator const &);
	virtual ~ProtocolManager() throw();

	int likelyMinimumMessageSize() const;

	const_iterator begin() const;
	const_iterator end() const;
	MessageLocation findMessage(XsProtocolType& type, const XsByteArray& raw) override;
	XsMessage convertToMessage(XsProtocolType& type, MessageLocation& location, const XsByteArray& raw) override;
	bool validateMessage(XsMessage const & message) const override;

	/*! \brief Adds the protocol handler
		\param[in] handler The protocol handler to add
		\returns The value type
	*/
	virtual value_type add(IProtocolHandler *handler);
	virtual bool remove(XsProtocolType type);
	virtual bool hasProtocol(XsProtocolType type) const;
	virtual void clear();

private:
	Communicator const & m_communicator;

	// mutable because the order of elements is optimized during findMessage to speedup future searches
	// but the findMessage method is conceptually const
	container_type m_protocolHandlers;
};

#endif
