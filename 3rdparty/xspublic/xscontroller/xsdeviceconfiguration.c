
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

#include "xsdeviceconfiguration.h"
#include <xstypes/xsmessage.h>
#include <xstypes/xsdeviceid.h>
#include <stdlib.h>
#include <memory.h>

#define INC_ALLOC	((void) 0)
#define INC_FREE	((void) 0)

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsDeviceConfiguration
	\brief Initializes the %XsDeviceConfiguration object
*/
void XsDeviceConfiguration_construct(XsDeviceConfiguration* thisPtr)
{
	memset(thisPtr, 0, sizeof(XsDeviceConfiguration));
}

/*! \relates XsDeviceConfiguration
	\brief Reinitializes the %XsDeviceConfiguration with space for \a numberOfDevices devices and copies them from \a src
	\details This function reinitializes the object reserving space for \a numberOfDevices devices in the buffer. \a size may
	be 0. If \a src is not null, \a numberOfDevices bytes from \a src will be copied.
	Previous data will be cleared and freed automatically.

	\param numberOfDevices : The number of devices for which memory should be allocated in the XsDeviceConfiguration
	\param src : A source XsDeviceConfiguration object that will be used to initialize the new object. May be null
*/
void XsDeviceConfiguration_assign(XsDeviceConfiguration* thisPtr, XsSize numberOfDevices, const XsDeviceConfiguration* src)
{
	if (numberOfDevices > thisPtr->m_numberOfDevices || numberOfDevices == 0)
	{
		XsDeviceConfiguration_destruct(thisPtr);
		if (numberOfDevices)
		{
			// init to size
			*((XsMtDeviceConfiguration**) &thisPtr->m_deviceInfo) = (XsMtDeviceConfiguration*) malloc(numberOfDevices*sizeof(XsMtDeviceConfiguration));
			INC_ALLOC;
		}
	}
	assert(numberOfDevices != 0 || thisPtr->m_deviceInfo == 0);

	*((uint16_t*) &thisPtr->m_numberOfDevices) = (uint16_t) numberOfDevices;

	if (src)
	{
		memcpy(&thisPtr->m_masterInfo, &src->m_masterInfo, sizeof(XsMasterDeviceConfiguration));
		if (numberOfDevices)
			memcpy(thisPtr->m_deviceInfo, src->m_deviceInfo, numberOfDevices*sizeof(XsMtDeviceConfiguration));
	}
}

/*! \relates XsDeviceConfiguration
	\brief Clears and frees data in the %XsDeviceConfiguration */
void XsDeviceConfiguration_destruct(XsDeviceConfiguration* thisPtr)
{
	if (thisPtr->m_deviceInfo)
	{
		// clear contents
		free(thisPtr->m_deviceInfo);
		INC_FREE;
	}
	// init to 0
	*((XsMtDeviceConfiguration**) &thisPtr->m_deviceInfo) = 0;
	*((uint16_t*) &thisPtr->m_numberOfDevices) = 0;
}

/*! \relates XsDeviceConfiguration
	\brief Copy the %XsDeviceConfiguration to \a copy */
void XsDeviceConfiguration_copy(XsDeviceConfiguration* copy, XsDeviceConfiguration const* src)
{
	if (copy == src)
		return;
	XsDeviceConfiguration_assign(copy, src->m_numberOfDevices, src);
}

/*! \relates XsDeviceConfiguration
	\brief Returns true if the XsDeviceConfiguration is empty
	\returns Returns true if the XsDeviceConfiguration is empty
*/
int XsDeviceConfiguration_empty(const XsDeviceConfiguration* thisPtr)
{
	return thisPtr->m_numberOfDevices == 0;
}

/*! \relates XsDeviceConfiguration
	\brief Sets up a XsDeviceConfiguration based upon \a msg

	\param msg : The XsMessage to use to set up the XsDeviceConfiguration object
*/
void XsDeviceConfiguration_readFromMessage(XsDeviceConfiguration* thisPtr, const XsMessage* msg)
{
	uint16_t i, nDevs;

	assert(XsMessage_getConstHeader(msg)->m_messageId == XMID_Configuration);

	nDevs = XsMessage_getDataShort(msg, 96);
	if (nDevs != thisPtr->m_numberOfDevices)
	{
		XsDeviceConfiguration_assign(thisPtr, nDevs, 0);
	}

	thisPtr->m_masterInfo.m_masterDeviceId = XsMessage_getDataLong(msg, 0);
	XsDeviceId deviceId = { thisPtr->m_masterInfo.m_masterDeviceId, XSDEVICEID_PRODUCT_CODE_INIT, 0, 0};
	if (XsDeviceId_isLegacyDeviceId(&deviceId))
	{
		thisPtr->m_masterInfo.m_samplingPeriod = XsMessage_getDataShort(msg, 4);
		thisPtr->m_masterInfo.m_outputSkipFactor = XsMessage_getDataShort(msg, 6);
	}
	else
	{
		thisPtr->m_masterInfo.m_masterDeviceId = XsMessage_getDataLong(msg, 0);
		thisPtr->m_masterInfo.m_masterDeviceId |= ((uint64_t)XsMessage_getDataLong(msg, 4) << 32);
	}
	memcpy(thisPtr->m_masterInfo.m_reserved1, XsMessage_getDataBuffer(msg, 8), 8);
	memcpy(thisPtr->m_masterInfo.m_date, XsMessage_getDataBuffer(msg, 16), 8);
	memcpy(thisPtr->m_masterInfo.m_time, XsMessage_getDataBuffer(msg, 24), 8);
	memcpy(thisPtr->m_masterInfo.m_productCode, XsMessage_getDataBuffer(msg, 32), 20);
	memcpy(thisPtr->m_masterInfo.m_reserved2, XsMessage_getDataBuffer(msg, 52), 44);

	for (i = 0; i < nDevs; ++i)
	{
		thisPtr->m_deviceInfo[i].m_deviceId = XsMessage_getDataLong(msg, 98+i*20);
		XsDeviceId deviceId = {thisPtr->m_deviceInfo[i].m_deviceId, XSDEVICEID_PRODUCT_CODE_INIT, 0, 0};
		if (XsDeviceId_isLegacyDeviceId(&deviceId))
		{
			thisPtr->m_deviceInfo[i].m_deviceId = (uint32_t)XsMessage_getDataLong(msg, 98+i*20);
			memcpy(thisPtr->m_deviceInfo[i].m_reserved, XsMessage_getDataBuffer(msg, 102 + i * 20), 8);
		}
		else
		{
			thisPtr->m_deviceInfo[i].m_deviceId = XsMessage_getDataLong(msg, 98+i*20);
			thisPtr->m_deviceInfo[i].m_deviceId |= ((uint64_t)XsMessage_getDataLong(msg, 102+i*20) << 32);
			memcpy(thisPtr->m_deviceInfo[i].m_reserved, XsMessage_getDataBuffer(msg, 106 + i * 20), 4);
		}
		thisPtr->m_deviceInfo[i].m_filterProfile = XsMessage_getDataShort(msg, 110+i*20);
		thisPtr->m_deviceInfo[i].m_fwRevMajor = XsMessage_getDataByte(msg, 112+i*20);
		thisPtr->m_deviceInfo[i].m_fwRevMinor = XsMessage_getDataByte(msg, 113+i*20);
		thisPtr->m_deviceInfo[i].m_fwRevRevision = XsMessage_getDataByte(msg, 114+i*20);
		thisPtr->m_deviceInfo[i].m_filterType = (char)XsMessage_getDataByte(msg, 115+i*20);
		thisPtr->m_deviceInfo[i].m_filterMajor = XsMessage_getDataByte(msg, 116+i*20);
		thisPtr->m_deviceInfo[i].m_filterMinor = XsMessage_getDataByte(msg, 117+i*20);
	}
}

/*! \relates XsDeviceConfiguration
	\brief Creates a message \a msg that represents the %XsDeviceConfiguration

	\param msg : An XsMessage that will represent the %XsDeviceConfiguration
*/
void XsDeviceConfiguration_writeToMessage(const XsDeviceConfiguration* thisPtr, XsMessage* msg)
{
	uint16_t i;
	XsMessageHeader* msgHeader;

	XsMessage_constructSized(msg, 98 + thisPtr->m_numberOfDevices * 20);
	msgHeader = XsMessage_getHeader(msg);
	msgHeader->m_messageId = XMID_Configuration;
	msgHeader->m_busId = XS_BID_MASTER;

	XsDeviceId deviceId = {thisPtr->m_masterInfo.m_masterDeviceId, XSDEVICEID_PRODUCT_CODE_INIT, 0, 0};
	if (XsDeviceId_isLegacyDeviceId(&deviceId))
	{
		XsMessage_setDataLong  (msg, (uint32_t)thisPtr->m_masterInfo.m_masterDeviceId, 0);
		XsMessage_setDataShort (msg, thisPtr->m_masterInfo.m_samplingPeriod, 4);
		XsMessage_setDataShort (msg, thisPtr->m_masterInfo.m_outputSkipFactor, 6);
	}
	else
	{
		XsMessage_setDataLong  (msg, (uint32_t)thisPtr->m_masterInfo.m_masterDeviceId & 0x00000000FFFFFFFF, 0);
		XsMessage_setDataLong  (msg, (uint32_t)((thisPtr->m_masterInfo.m_masterDeviceId & 0xFFFFFFFF00000000) >> 32), 4);
	}

	XsMessage_setDataBuffer(msg, thisPtr->m_masterInfo.m_reserved1, 8, 8);
	XsMessage_setDataBuffer(msg, thisPtr->m_masterInfo.m_date, 8, 16);
	XsMessage_setDataBuffer(msg, thisPtr->m_masterInfo.m_time, 8, 24);
	XsMessage_setDataBuffer(msg, thisPtr->m_masterInfo.m_productCode, 20, 32);
	XsMessage_setDataBuffer(msg, thisPtr->m_masterInfo.m_reserved2, 44, 52);
	XsMessage_setDataShort(msg, thisPtr->m_numberOfDevices, 96);

	for (i = 0; i < thisPtr->m_numberOfDevices; ++i)
	{
		XsDeviceId deviceId = {thisPtr->m_deviceInfo[i].m_deviceId, XSDEVICEID_PRODUCT_CODE_INIT, 0, 0};
		if (XsDeviceId_isLegacyDeviceId(&deviceId))
		{
			XsMessage_setDataLong (msg, (uint32_t)thisPtr->m_deviceInfo[i].m_deviceId, 98+i*20);
			XsMessage_setDataBuffer(msg, thisPtr->m_masterInfo.m_reserved1, 8, 102+i*20);
		}
		else
		{
			XsMessage_setDataLong  (msg, (uint32_t)thisPtr->m_deviceInfo[i].m_deviceId & 0x00000000FFFFFFFF, 98+i*20);
			XsMessage_setDataLong  (msg, (uint32_t)((thisPtr->m_deviceInfo[i].m_deviceId & 0xFFFFFFFF00000000) >> 32), 102+i*20);
			XsMessage_setDataBuffer(msg, thisPtr->m_masterInfo.m_reserved1, 4, 106+i*20);
		}
		XsMessage_setDataShort(msg, thisPtr->m_deviceInfo[i].m_filterProfile, 110+i*20);
		XsMessage_setDataByte (msg, thisPtr->m_deviceInfo[i].m_fwRevMajor, 112+i*20);
		XsMessage_setDataByte (msg, thisPtr->m_deviceInfo[i].m_fwRevMinor, 113+i*20);
		XsMessage_setDataByte (msg, thisPtr->m_deviceInfo[i].m_fwRevRevision, 114+i*20);
		XsMessage_setDataByte (msg, thisPtr->m_deviceInfo[i].m_filterType, 115+i*20);
		XsMessage_setDataByte (msg, thisPtr->m_deviceInfo[i].m_filterMajor, 116+i*20);
		XsMessage_setDataByte (msg, thisPtr->m_deviceInfo[i].m_filterMinor, 117+i*20);
	}
	XsMessage_recomputeChecksum(msg);
}

/*! \relates XsDeviceConfiguration
	\brief Returns a (naive) bus ID if \a deviceId is available in this configuration, 0 otherwise

	\note The bus ID may not be valid for device communications

	\param deviceId the device ID to look for
	\returns the bus ID that matches \a deviceId
*/
XsSize XsDeviceConfiguration_findDevice(const XsDeviceConfiguration* thisPtr, const XsDeviceId* deviceId)
{
	uint16_t i;
	for (i = 0; i < thisPtr->m_numberOfDevices; ++i)
	{
		if (thisPtr->m_deviceInfo[i].m_deviceId == deviceId->m_deviceId)
			return i + 1;
	}
	return 0;
}
 /*! @} */
