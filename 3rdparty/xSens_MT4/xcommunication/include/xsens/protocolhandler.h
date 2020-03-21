/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#ifndef PROTOCOLHANDLER_H
#define PROTOCOLHANDLER_H

#include "iprotocolhandler.h"
#include "xcommunicationconfig.h"

//--------------------------------------------------------------------------------
class ProtocolHandler : public virtual IProtocolHandler
{
   public:
	ProtocolHandler();
	~ProtocolHandler() override;

	MessageLocation findMessage(
		XsMessage& rcv, const XsByteArray& raw) const override;
	int minimumMessageSize() const override;
	int maximumMessageSize() const override;
	int type() const override;
	static int composeMessage(XsByteArray& raw, const XsMessage& msg);

	XSENS_DISABLE_COPY(ProtocolHandler)
};

//--------------------------------------------------------------------------------

#endif  // file guard
