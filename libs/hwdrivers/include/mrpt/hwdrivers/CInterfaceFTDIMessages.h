/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CInterfaceFTDIMessages_H
#define CInterfaceFTDIMessages_H

#include <mrpt/hwdrivers/CInterfaceFTDI.h>
#include <mrpt/utils/CMessage.h>

namespace mrpt
{
	namespace hwdrivers
	{
		/** Since MRPT 0.9.1 this class has no extra functionality, since the methods for sending/receiving messages are not in CStream.
		 */
		typedef CInterfaceFTDI  CInterfaceFTDIMessages;

	} // end of namespace
} // end of namespace

#endif


