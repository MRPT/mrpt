
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

#ifndef IPROTOCOLMANAGER_H
#define IPROTOCOLMANAGER_H

#include <xstypes/xsmessage.h>
#include "messagelocation.h"
#include "xsprotocoltype.h"

/*! \brief Interface class for protocol manager
\details Describes the interfaces of a manager of different protocols \sa ProtocolHandler
*/
class IProtocolManager
{
public:

	//! \brief Destructor
	virtual ~IProtocolManager() {}

	/*! \brief Will let all supported protocols attempt finding a raw message in the given raw data
		\param type: The protocol type to us
		\param raw: The input raw byte array in which to look for a message
		\returns A MessageLocation object describing the best possible found message \sa MessageLocation
	*/
	virtual MessageLocation findMessage(XsProtocolType& type, const XsByteArray& raw) = 0;

	/*! \brief Converts \a raw data using \a location into a %XsMessage object.
		\param type: The protocol type to use.
		\param location: The location of a message to convert from \a raw data.
		\param raw: The raw byte stream.
		\returns A %XsMessage object that was converted from raw byte stream.
	*/
	virtual XsMessage convertToMessage(XsProtocolType& type, MessageLocation& location, const XsByteArray& raw) = 0;

	/*! \brief Performs a sanity check on the given message
		\param msg: The message to check
		\returns true if the message passes the protocol managers sanity checks
	*/
	virtual bool validateMessage(XsMessage const &msg) const = 0;
};

#endif
