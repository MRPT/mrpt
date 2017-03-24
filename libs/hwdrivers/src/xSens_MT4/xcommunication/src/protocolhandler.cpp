/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "protocolhandler.h"
#include <xsens/xsmessage.h>
#include <xsens/xsresultvalue.h>
#include <iomanip>

/*! \class ProtocolHandler
	\brief Message protocol handling class

	This class' purpose is to get valid messages according to its protocol from the raw
	data that is supplied to it. The default implementation (ProtocolHandler) implements
	the Xsens message protocol. To use a different protocol, overload the findMessage function.

	The class is intended to be state-less with respect to the data it handles.
*/

//! Default constructor
ProtocolHandler::ProtocolHandler()
{}

//! Destructor
ProtocolHandler::~ProtocolHandler()
{}

/*! \brief Compute the expected message size given a possibly incomplete message
	\details When analyzing a message with incomplete size information, the function
	will return the minimum size of the message given the information that is available
*/
int expectedMessageSize(const unsigned char* buffer, int sz)
{
	const XsMessageHeader* hdr = (const XsMessageHeader*) buffer;
	if (sz < 4)
		return XS_LEN_MSGHEADERCS;	// no size information available at all, return a minimum message

	// we have size information
	if (hdr->m_length == XS_EXTLENCODE)
	{
		if (sz < 6)
			return XS_EXTLENCODE + XS_LEN_MSGEXTHEADERCS;	// typical minimum size at which point extended size is needed

		return XS_LEN_MSGEXTHEADERCS + ((uint32_t) hdr->m_datlen.m_extended.m_length.m_high * 256 + (uint32_t) hdr->m_datlen.m_extended.m_length.m_low);
	}
	return XS_LEN_MSGHEADERCS + (uint32_t) (hdr->m_length);
}

/*! \copydoc IProtocolHandler::findMessage
	\todo Since the assumption is that we receive a stream of valid messages without garbage, the scan
	is implemented in a rather naive and simple way. If we can expect lots of garbage in the data
	stream, this should probably be looked into.
*/
MessageLocation ProtocolHandler::findMessage(XsMessage& rcv, const XsByteArray& raw) const
{
	JLTRACE(gJournal, "Entry");
	MessageLocation rv(-1,0);
	rcv.clear();

	int bufferSize = (int) raw.size();
	if (bufferSize == 0)
		return rv;
	
	const unsigned char* buffer = raw.data();

	// loop through the buffer to find a preamble
	for (int pre = 0; pre < bufferSize; ++pre)
	{
		if (buffer[pre] == XS_PREAMBLE)
		{
			JLTRACE(gJournal, "Preamble found at " << pre);
			// we found a preamble, see if we can read a message from here
			if (rv.m_startPos == -1)
				rv.m_startPos = (int32_t) pre;
			int remaining = bufferSize-pre;	// remaining bytes in buffer INCLUDING preamble

			if (remaining < XS_LEN_MSGHEADERCS)
			{
				JLTRACE(gJournal, "Not enough header data read");
				if (rv.m_startPos != pre)
					continue;
				rv.m_size = -expectedMessageSize(&buffer[pre], remaining);
				return rv;
			}

			// read header
			const uint8_t* msgStart = &(buffer[pre]);
			const XsMessageHeader* hdr = (const XsMessageHeader*) msgStart;
			if (hdr->m_length == XS_EXTLENCODE)
			{
				if (remaining < XS_LEN_MSGEXTHEADERCS)
				{
					JLTRACE(gJournal, "Not enough extended header data read");
					if (rv.m_startPos != pre)
						continue;
					rv.m_size = -expectedMessageSize(&buffer[pre], remaining);
					return rv;
				}
			}
			else if (hdr->m_busId == 0 && hdr->m_messageId == 0)
			{
				// found 'valid' message that isn't actually valid... happens inside GPS raw data
				// skip to next preamble
				continue;
			}

			// check the reported size
			int target = expectedMessageSize(&buffer[pre], remaining);

			JLTRACE(gJournal, "Bytes in buffer=" << remaining << ", full target = " << target);
			if (target > (XS_LEN_MSGEXTHEADERCS + XS_MAXDATALEN))
			{
				// skip current preamble
				JLALERT(gJournal, "Invalid message length: " << target);
				rv.m_startPos = -1;
				continue;
			}

			if (remaining < target)
			{
				// not enough data read, skip current preamble
				JLTRACE(gJournal, "Not enough data read: " << remaining << " / " << target);
				if (rv.m_size == 0)
					rv.m_size = -target;
				continue;
			}

			// we have read enough data to fulfill our target so we'll try to parse the message
			// and check the checksum
			//if (rcv->loadFromString(msgStart, (uint16_t) target) == XRV_OK)
			if (rcv.loadFromString(msgStart, (uint16_t)target))
			{
				JLTRACE(gJournal,
						"OK, size = " << (int) rcv.getTotalMessageSize()
						<< std::hex << std::setfill('0')
						<< " First bytes " << std::setw(2) << (int) msgStart[0]
						<< " " << std::setw(2) << (int) msgStart[1]
						<< " " << std::setw(2) << (int) msgStart[2]
						<< " " << std::setw(2) << (int) msgStart[3]
						<< " " << std::setw(2) << (int) msgStart[4]
						<< std::dec << std::setfill(' '));
				rv.m_size = (int) rcv.getTotalMessageSize();
				rv.m_startPos = pre;	// we do this again here because this may not be the first preamble encountered (the check for -1 at the start of the loop is necessary)
				return rv;
			}

			// we could not read the message, clear message and try next preamble
			rcv.clear();
			if (rv.m_startPos == pre)
			{
				rv.m_startPos = -1;
				JLALERT(gJournal,
					"Invalid checksum"
					<< std::hex << std::setfill('0')
					<< " First bytes " << std::setw(2) << (int) msgStart[0]
					<< " " << std::setw(2) << (int) msgStart[1]
					<< " " << std::setw(2) << (int) msgStart[2]
					<< " " << std::setw(2) << (int) msgStart[3]
					<< " " << std::setw(2) << (int) msgStart[4]
					<< std::dec << std::setfill(' '));
			}
		}
	}
	JLTRACE(gJournal, "Exit");
	return rv;
}
/*! \brief Returns the minimum size of a valid message of this protocol including preambles and checksums */
int ProtocolHandler::minimumMessageSize() const
{
	return XS_LEN_MSGHEADERCS;	// minimum size of xsens xbus protocol message
}

/*! \brief Returns the maximum size of a valid message of this protocol including preambles and checksums */
int ProtocolHandler::maximumMessageSize() const
{
	return XS_LEN_MSGEXTHEADERCS+XS_MAXDATALEN;	// maximum size of xsens xbus protocol message
}

/*! \brief Compose a message for transmission
	\param raw The raw byte array to be constructed from the message
	\param msg The message to translate into a raw byte array
	\returns The size of the generated byte array
	\todo Generalize this method -> IProtocolHandler
*/
int ProtocolHandler::composeMessage(XsByteArray& raw, const XsMessage& msg)
{
	if (msg.getTotalMessageSize() < 5)	// minimum size of an xsens message including envelope is 5 bytes
		return -1;

	raw.assign(msg.getTotalMessageSize(), msg.getMessageStart());
	return (int) raw.size();
}

int ProtocolHandler::type() const
{
	return 0; // XPT_Xbus;
}
