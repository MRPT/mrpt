/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef IPROTOCOLHANDLER_H
#define IPROTOCOLHANDLER_H

#include "messagelocation.h"
#include <xsens/xsmessage.h>
#include <xsens/xsbytearray.h>

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
		\details This function scans \a raw for a sequence of bytes that it can convert into an
		%XsMessage object. It returns the location and total byte size of the message so that the
		caller can remove those bytes from the stream. The return value can also describe that a 
		partial message has been found. Return values:
		\li \a startpos >= 0 and \a size > 0: A full message with \a size has been found at \a startpos.
		\li \a startpos >= 0 and \a size == 0: The start of a message has been found at \a startpos, but the size could not yet be determined.
		\li \a startpos >= 0 and \a size < 0: The start of a message has been found at \a startpos, and the size of the full message is at least \a -size.
		\li \a startpos < 0: No messages have been found.

		\param rcv If a message is read, it will be put in this object.
		\param raw The raw byte stream to analyze.
		\returns A %MessageLocation object that describes what was found.
	*/
	virtual MessageLocation findMessage(XsMessage& rcv, const XsByteArray& raw) const = 0;
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
};

//--------------------------------------------------------------------------------

#endif // file guard
