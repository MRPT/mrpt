/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#include "xsportinfo.h"
#include <ctype.h>
#include <string.h>	// strlen
#include <stdlib.h> 	// atoi

/*! \class XsPortInfo
	\brief Contains a descriptor for opening a communication port to an Xsens device.
*/
/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsPortInfo
	\brief Initializes the object to the empty state
*/	
void XsPortInfo_clear(XsPortInfo* thisPtr)
{
	thisPtr->m_baudrate = XBR_Invalid;
	thisPtr->m_deviceId.m_deviceId = 0;
	thisPtr->m_portName[0] = '\0';
}

/*! \relates XsPortInfo
	\brief Returns true if the XsPortInfo object is empty
*/	
int XsPortInfo_empty(const struct XsPortInfo* thisPtr)
{
	return (thisPtr->m_portName[0] == '\0');
}

/*! \relates XsPortInfo
    \brief The port number
	\returns Returns the port number
	\note Available on Windows only
*/
int XsPortInfo_portNumber(const struct XsPortInfo* thisPtr)
{
	size_t i;

	if (XsPortInfo_empty(thisPtr))
		return 0;

	for (i = 0; i < strlen(thisPtr->m_portName); i++) {
		if (isdigit(thisPtr->m_portName[i])) {
			return atoi(&thisPtr->m_portName[i]);
		}
	}
	return 0;
}

/*! \relates XsPortInfo
	\brief Returns true if this port info object contains a USB device
 */
int XsPortInfo_isUsb(const struct XsPortInfo* thisPtr)
{
#ifdef XSENS_WINDOWS
	return strncmp("\\\\?\\usb", thisPtr->m_portName, 7) == 0;
#else
	return strncmp("USB", thisPtr->m_portName, 3) == 0;
#endif
}

/*! \relates XsPortInfo
	\brief The usb bus
	\returns Returns the Usb bus number
	\note Available on Linux only
*/
int XsPortInfo_usbBus(const struct XsPortInfo* thisPtr)
{
#ifndef XSENS_WINDOWS
	if (XsPortInfo_isUsb(thisPtr))
		return atoi(&thisPtr->m_portName[3]);
#endif
	return 0;
}

/*! \relates XsPortInfo
	\brief The usb address
	\returns Returns the usb address
	\note Available on Linux only
*/
int XsPortInfo_usbAddress(const struct XsPortInfo* thisPtr)
{
#ifndef XSENS_WINDOWS
	if (XsPortInfo_isUsb(thisPtr))
		return atoi(&thisPtr->m_portName[7]);
#endif
	return 0;
}

/*! \brief Swap the contents of \a a with those of \a b
*/
void XsPortInfo_swap(struct XsPortInfo* a, struct XsPortInfo* b)
{
	int i;
	char c;

	XsBaudRate t = a->m_baudrate;
	a->m_baudrate = b->m_baudrate;
	b->m_baudrate = t;

	XsDeviceId_swap(&a->m_deviceId, &b->m_deviceId);

	for (i = 0; i < 256; ++i)
	{
		c = a->m_portName[i];
		a->m_portName[i] = b->m_portName[i];
		b->m_portName[i] = c;
	}
}

/*! @} */
