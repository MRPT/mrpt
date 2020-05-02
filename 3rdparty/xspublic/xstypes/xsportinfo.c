
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
	thisPtr->m_deviceId.m_productCode[0] = '\0';
	thisPtr->m_portName[0] = '\0';
	thisPtr->m_linesOptions = XPLO_All_Ignore;
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
	return strncmp("USB", thisPtr->m_portName, 3) == 0; // libusb devices start with USB
#endif
}

/*!
 * \relates XsPortInfo
 * \brief Returns true if this port info object contains a network device
 */
int XsPortInfo_isNetwork(const struct XsPortInfo* thisPtr)
{
	return strncmp("NET:", thisPtr->m_portName, 4) == 0;
}

/*!
 * \relates XsPortInfo
 * \brief Returns the network service name of this port
 */
const char* XsPortInfo_networkServiceName(const struct XsPortInfo* thisPtr)
{
	return &thisPtr->m_portName[4];
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
#else
	(void) thisPtr;
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
#else
	(void) thisPtr;
#endif
	return 0;
}

/*! \brief Swap the contents of \a a with those of \a b
*/
void XsPortInfo_swap(struct XsPortInfo* a, struct XsPortInfo* b)
{
	int i;
	char c;
	XsPortLinesOptions pLineOpts;

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

	pLineOpts = a->m_linesOptions;
	a->m_linesOptions = b->m_linesOptions;
	b->m_linesOptions = pLineOpts;
}

/*! @} */
