
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

#include "nmea_protocolhandler.h"
#include "xscontrollerconfig.h"

namespace nmea
{

const char PREAMBLE = '$';
const char PREAMBLE_TSS2 = ':';
const char PREAMBLE_EM1000 = '\0';
const unsigned char AFTER_PREAMBLE_EM1000 = '\x90';
const char CR = '\r';
const char LF = '\n';

/*! \class ProtocolHandler
	\brief The protocol handler for a different NMEA data types
*/

/*! \brief Default constructor
*/
ProtocolHandler::ProtocolHandler()
{
}

/*! \brief Deafult destructor
*/
ProtocolHandler::~ProtocolHandler() throw()
{
}

/*! \copydoc IProtocolHandler::findMessage
*/
MessageLocation ProtocolHandler::findMessage(XsProtocolType& type, const XsByteArray& raw) const
{
	type = XPT_Nmea;
	MessageLocation location(-1, 0, -1, 0);

	int bufferSize = (int)raw.size();
	if (bufferSize == 0)
		return location;

	const unsigned char* buffer = raw.data();

	bool foundPreamble = false;
	bool foundEnd = false;
	int start = 0;
	int end = 0;
	for (int i = 0; i < bufferSize; i++)
	{
		if (!foundPreamble)
		{
			if ((buffer[i] == PREAMBLE) || (buffer[i] == PREAMBLE_TSS2) || (buffer[i] == PREAMBLE_EM1000))
			{
				foundPreamble = true;
				start = i;
			}
		}
		if (foundPreamble)
		{
			if ((buffer[i] == LF) && (buffer[i - 1] == CR))
			{
				foundEnd = true;
				end = i + 1;
			}
			else if ((buffer[i] == AFTER_PREAMBLE_EM1000) && (buffer[i - 1] == PREAMBLE_EM1000)) // special case EM1000
			{
				if ((i - start) == 1 && (i + 9) < bufferSize)
				{
					foundEnd = true;
					end = i + 9;
				}
				else
				{
					foundPreamble = false;
				}
			}
		}
		if (foundPreamble && foundEnd)
			break;
	}

	location.m_startPos = start;
	location.m_size = end - start;

	return location;
}

/*! \copydoc IProtocolHandler::convertToMessage
*/
XsMessage ProtocolHandler::convertToMessage(MessageLocation& location, const XsByteArray& raw) const
{
	(void)location;
	(void)raw;
	return XsMessage();
}

/*! \returns The minimum message size
*/
int ProtocolHandler::minimumMessageSize() const
{
	return MINIMUM_MESSAGE_SIZE;
}

/*! \returns The maximum message size
*/
int ProtocolHandler::maximumMessageSize() const
{
	return MAXIMUM_MESSAGE_SIZE;
}

/*! \returns The type of the protocol */
int ProtocolHandler::type() const
{
	return XPT_Nmea;
}

}
