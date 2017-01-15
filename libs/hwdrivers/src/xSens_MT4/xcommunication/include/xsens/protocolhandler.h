/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef PROTOCOLHANDLER_H
#define PROTOCOLHANDLER_H

#include "xcommunicationconfig.h"
#include "iprotocolhandler.h"

//--------------------------------------------------------------------------------
class ProtocolHandler : public virtual IProtocolHandler
{
public:
	ProtocolHandler();
	virtual ~ProtocolHandler();

	virtual MessageLocation findMessage(XsMessage& rcv, const XsByteArray& raw) const;
	virtual int minimumMessageSize() const;
	virtual int maximumMessageSize() const;
	virtual int type() const;
	static int composeMessage(XsByteArray& raw, const XsMessage& msg);

	XSENS_DISABLE_COPY(ProtocolHandler)
};

//--------------------------------------------------------------------------------

#endif	// file guard
