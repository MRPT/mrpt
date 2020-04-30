
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

#ifndef IPROTOCOLHANDLER_H
#define IPROTOCOLHANDLER_H

#include "messagelocation.h"
#include "xsprotocoltype.h"
#include <xstypes/xsmessage.h>
#include <xstypes/xsbytearray.h>

//--------------------------------------------------------------------------------
/*! \brief Interface class for protocol handlers
	\details Describes the interfaces of the protocol handling classes. The protocol handlers are
	used to convert a binary data stream into XsMessage objects.
*/
class IProtocolHandler
{
public:
	//! \brief Destructor
	virtual ~IProtocolHandler() {}

	/*! \brief Find the first message in the \a raw byte stream
		\details This function scans \a raw for a sequence of bytes that can contain a full message.
		It returns the location and total byte size of the message so that the
		caller can remove those bytes from the stream. The return value can also describe that a
		partial message has been found. Return values:
		\li \a startpos >= 0 and \a size > 0: A full message with \a size has been found at \a startpos.
		\li \a startpos >= 0 and \a size == 0: The start of a message has been found at \a startpos, but the size could not yet be determined.
		\li \a startpos >= 0 and \a size < 0: The start of a message has been found at \a startpos, and the size of the full message is at least \a -size.
		\li \a startpos < 0: No messages have been found.

		\param type The protocol type that was used.
		\param raw The raw byte stream to analyze.
		\returns A %MessageLocation object that describes what was found.
	*/
	virtual MessageLocation findMessage(XsProtocolType& type, const XsByteArray& raw) const = 0;

	/*! \brief Converts \a raw data using \a location into a %XsMessage object.
		\param location The location of a message to convert from \a raw data.
		\param raw The raw byte stream.
		\returns A %XsMessage object that was converted from raw byte stream.
	*/
	virtual XsMessage convertToMessage(MessageLocation& location, const XsByteArray& raw) const = 0;

	/*! \brief Returns the minimum size of a valid message
		\details This value may differ for different protocols, but is always at least 1.
		\returns The minimum size of a valid message for the protocol.
	*/
	virtual int minimumMessageSize() const = 0;

	/*! \brief Returns the maximum size of a valid message
		\details This value may differ for different protocols.
		\returns The maximum size of a valid message for the protocol.
	*/
	virtual int maximumMessageSize() const = 0;

	/*! \brief Returns the type of the protocol handler
		\details Each protocol handler has a locally unique id that can be used for instantiation of
		the correct protocol handler.
		\returns The type id of the protocol handler.
	*/
	virtual int type() const = 0;

	/*! \brief Tells the protocol handler to ignore/expand its maximum message size
		\details This is mostly used when reading from a file that is known to contain correct data.
		Please note that the protocol handler can decide to ignore this call, which is what the default
		implementation does.
		\param ignore Set to true to ignore the maximum message size check.
	*/
	virtual void ignoreMaximumMessageSize(bool ignore)
	{
		(void) ignore;
	}
};

//--------------------------------------------------------------------------------

#endif
