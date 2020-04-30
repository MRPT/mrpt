
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

#include "xsdeviceid.h"
#include "xsstring.h"
#include "xsdid.h"
#include <stdio.h>
#include <stdlib.h>

/*! \class XsDeviceId
	\brief Contains an Xsens device ID and provides operations for determining the type of device
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \returns The legacy bit used to identify legacy or new XsDeviceId format */
uint64_t XsDeviceId_legacyBit(const struct XsDeviceId *thisPtr)
{
	(void)thisPtr;
	return XS_DID64_BIT;
}

/*! \brief Test if the device ID represents a legacy device identification */
int XsDeviceId_isLegacyDeviceId(const struct XsDeviceId *thisPtr)
{
	return ((thisPtr->m_deviceId & 0x0000000080000000) == 0);
}

/*! \brief Test if the device ID represents an Mti 1-series device */
int XsDeviceId_isMtiX(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X_MPU);
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;
		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		return ((deviceFamily != 0) && (deviceFamily < 10));
	}
}

/*! \brief Test if the device ID represents an Mti 10-series device */
int XsDeviceId_isMtiX0(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X0);
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;
		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		return ((deviceFamily != 0) && ((deviceFamily >= 10) && (deviceFamily < 100)));
	}
}

/*! \brief Test if the device ID represents an Mti 100-series device */
int XsDeviceId_isMtiX00(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_MT_X00);
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;
		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		if ((deviceFamily != 0) && (deviceFamily >= 100 && deviceFamily <= 300))
		{
			return 1;
		}
		else if (memcmp(thisPtr->m_productCode, "MTi-G-", 6) == 0)
		{
			deviceFamily = atoi(&thisPtr->m_productCode[6]);
			if ((deviceFamily != 0) && (deviceFamily >= 100))
				return 1;
		}
	}
	return 0;
}

/*! \brief Test if the device ID represents an Mtig 700 device */
int XsDeviceId_isMtigX00(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700) &&
			((thisPtr->m_deviceId &~XS_DID_TYPEL_COMM_MASK) < XS_DID_MK4TYPE_MT_710_RANGE_START);
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-G-", 6) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[6]);
		return ((deviceFamily != 0) && (deviceFamily == 700));
	}
}

/*! \brief Test if the device ID represents an Mtig 710 device */
int XsDeviceId_isMtigX10(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700) &&
			((thisPtr->m_deviceId & ~XS_DID_TYPEL_COMM_MASK) >= XS_DID_MK4TYPE_MT_710_RANGE_START);
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-G-", 6) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[6]);
		return ((deviceFamily != 0) && (deviceFamily == 710));
	}
}

/*! \brief Test if the device ID represents an MTi-600 series device */
int XsDeviceId_isMti6X0(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return 0;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		return ((deviceFamily != 0) && ((deviceFamily >= 600) && (deviceFamily < 700)));
	}
}

/*! \brief Test if the device ID represents an Glove series device */
int XsDeviceId_isGlove(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return 0;
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "Glove", 5) != 0)
			return 0;

		return 1;
	}
}

/*! \brief Test if this device ID represents an MTw */
int XsDeviceId_isMtw(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtw2(thisPtr);
}

/*! \brief Test if this device ID represents an MTw2. */
int XsDeviceId_isMtw2(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return XS_DID_MTW2(thisPtr->m_deviceId);
	}
	else
	{
		return (memcmp(thisPtr->m_productCode, "MTw2", 4) == 0);
	}
}

/*! \brief Test if this device ID represents an MTx */
int XsDeviceId_isMtx(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtx2(thisPtr);
}

/*! \brief Test if this device ID represents an MTx2 */
int XsDeviceId_isMtx2(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return XS_DID_MTX2(thisPtr->m_deviceId);
	}
	else
	{
		return (memcmp(thisPtr->m_productCode, "MTx2", 4) == 0);
	}
}

/*! \brief Test if this device ID represents a bodypack device. */
int XsDeviceId_isBodyPack(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return XS_DID_BODYPACK(thisPtr->m_deviceId) || (thisPtr->m_deviceId == XS_DID_ABMCLOCKMASTER);
	}
	else
	{
		return (memcmp(thisPtr->m_productCode, "BodyPack", 8) == 0);
	}
}

/*! \brief Test if this device ID represents a Wireless Master device (Awinda Station, Awinda Dongle, Awinda OEM). */
int XsDeviceId_isWirelessMaster(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return ((thisPtr->m_deviceId & XS_DID_TYPEH_MASK) == XS_DID_TYPEH_AWINDAMASTER) &&
				!XsDeviceId_isBodyPack(thisPtr) &&
				!XsDeviceId_isSyncStationX(thisPtr);
	}
	else
	{
		return (memcmp(thisPtr->m_productCode, "AW-", 3) == 0);
	}
}

/*! \brief Test if this device ID represents an awinda device. */
int XsDeviceId_isAwindaX(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda2(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda Station.
*/
int XsDeviceId_isAwindaXStation(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda2Station(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda Dongle. */
int XsDeviceId_isAwindaXDongle(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda2Dongle(thisPtr);
}

/*! \brief Test if this device ID represents an Awinda OEM board. */
int XsDeviceId_isAwindaXOem(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isAwinda2Oem(thisPtr);
}

/*! \brief Test if this device ID represents an awinda2 device. */
int XsDeviceId_isAwinda2(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return XS_DID_AWINDA2(thisPtr->m_deviceId);
	}
	else
	{
		return (memcmp(thisPtr->m_productCode, "AW-", 3) == 0);
	}
}

/*! \brief Test if this device ID represents an Awinda2 Station.
*/
int XsDeviceId_isAwinda2Station(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return XS_DID_AWINDA2_STATION(thisPtr->m_deviceId);
	}
	else
	{
		return memcmp(thisPtr->m_productCode, "AW-A2", 5) == 0;
	}
}

/*! \brief Test if this device ID represents an Awinda2 Dongle. */
int XsDeviceId_isAwinda2Dongle(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return XS_DID_AWINDA2_DONGLE(thisPtr->m_deviceId);
	}
	else
	{
		return memcmp(thisPtr->m_productCode, "AW-DNG2", 7) == 0;
	}
}

/*! \brief Test if this device ID represents an Awinda2 OEM board. */
int XsDeviceId_isAwinda2Oem(const struct XsDeviceId* thisPtr)
{
	return XS_DID_AWINDA2_OEM(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a SyncStation. */
int XsDeviceId_isSyncStationX(const struct XsDeviceId* thisPtr)
{
	return XS_DID_SYNCSTATION(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a SyncStation v2. */
int XsDeviceId_isSyncStation2(const struct XsDeviceId* thisPtr)
{
	return XS_DID_SYNCSTATION2(thisPtr->m_deviceId);
}

/*! \brief Test if this device ID represents a Hardware In the Loop test device. */
int XsDeviceId_isHilDevice(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return 0;
	}
	else
	{
		if (strcmp(thisPtr->m_productCode, "HILDEVICE") != 0)
			return 0;
		return 1;
	}
}

/*! \brief Test if this device ID represents an IMU. */
int XsDeviceId_isImu(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_1_MPU) ||
			((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_10) ||
			((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_100));
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = thisPtr->m_productCode[4] - '0';
		if (deviceFamily == 0)
			return 0;

		if (deviceFamily == 6)
			deviceFamily = thisPtr->m_productCode[5] - '0';

		return (deviceFamily == 1);
	}
}

/*! \brief Test if this device ID represents a VRU. */
int XsDeviceId_isVru(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_2_MPU) ||
			((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_20) ||
			((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_200));
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = thisPtr->m_productCode[4] - '0';
		if (deviceFamily == 0)
			return 0;

		if (deviceFamily == 6)
			deviceFamily = thisPtr->m_productCode[5] - '0';

		return (deviceFamily == 2);
	}
}

/*! \brief Test if this device ID represents an AHRS. */
int XsDeviceId_isAhrs(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_3_MPU) ||
			((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_30) ||
			((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_300));
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = thisPtr->m_productCode[4] - '0';
		if (deviceFamily == 0)
			return 0;

		if (deviceFamily == 6)
			deviceFamily = thisPtr->m_productCode[5] - '0';

		return (deviceFamily == 3);
	}
}

/*! \brief Test if this device ID represents an GNSS (capable) device. */
int XsDeviceId_isGnss(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		return (((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_700) ||
			((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_7_MPU));
	}
	else
	{
		if (memcmp(thisPtr->m_productCode, "MTi-", 4) != 0)
			return 0;

		int deviceFamily = atoi(&thisPtr->m_productCode[4]);
		if (deviceFamily == 7)
		{
			return 1;
		}
		else if (deviceFamily == 670)
		{
			return 1;
		}
		else
		{
			if (memcmp(thisPtr->m_productCode, "MTi-G-", 6) != 0)
				return 0;

			deviceFamily = atoi(&thisPtr->m_productCode[6]);
			return (deviceFamily == 700 || deviceFamily == 710);
		}
	}
}

/*! \brief Test if this device ID represents any of the container devices such as Bodypack and Awinda Station
*/
int XsDeviceId_isContainerDevice(const struct XsDeviceId* thisPtr)
{
	return	XsDeviceId_isBodyPack(thisPtr) ||
		XsDeviceId_isWirelessMaster(thisPtr);
}

/*! \brief Test if this device ID represents an MT device (any Mti, Mtig, Mtx or Mtw) */
int XsDeviceId_isMt(const struct XsDeviceId* thisPtr)
{
	return (XsDeviceId_isMti(thisPtr) || XsDeviceId_isMtig(thisPtr) || XsDeviceId_isMtw(thisPtr) || XsDeviceId_isMtx(thisPtr));
}

/*! \brief Test if this device ID represents an MTi device (1, 10 or 100 series, 1 includes MTi-7) */
int XsDeviceId_isMti(const struct XsDeviceId* thisPtr)
{
	return (XsDeviceId_isMtiX(thisPtr) || XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr) || XsDeviceId_isMti6X0(thisPtr));
}

/*! \brief Test if this device ID represents an MTig device (700 or 710 series) */
int XsDeviceId_isMtig(const struct XsDeviceId* thisPtr)
{
	return (XsDeviceId_isMtigX00(thisPtr) || XsDeviceId_isMtigX10(thisPtr));
}

/*! \brief Test if this device ID represents an Mk4 generation MT device */
int XsDeviceId_isMtMark4(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return (XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr)) && ((thisPtr->m_deviceId & XS_DID_TYPEL_MK5) != XS_DID_TYPEL_MK5);
	else
		return (XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr)) && thisPtr->m_hardwareVersion < 0x300;
}

/*! \brief Test if this device ID represents an Mk5 generation MT device */
int XsDeviceId_isMtMark5(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return (XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr)) && ((thisPtr->m_deviceId & XS_DID_TYPEL_MK5) == XS_DID_TYPEL_MK5);
	else
		return (XsDeviceId_isMtiX0(thisPtr) || XsDeviceId_isMtiX00(thisPtr)) && thisPtr->m_hardwareVersion >= 0x300;
}

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

/*! \brief Get a string with a readable representation of this device ID. */
void XsDeviceId_toString(const XsDeviceId* thisPtr, XsString* str)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		char device[9];
		sprintf(device, "%08" PRINTF_INT32_MODIFIER "X", (uint32_t)thisPtr->m_deviceId);
		XsString_assign(str, 8, device);
	}
	else
	{
		char device[17];
		XsSize n = (XsSize) (ptrdiff_t) sprintf(device, "%010" PRINTF_INT64_MODIFIER "X", thisPtr->m_deviceId);
		XsString_assign(str, n, device);
	}
}

/*! \brief Read a device ID from the supplied string.
	\param str The string to interpret
*/
void XsDeviceId_fromString(XsDeviceId* thisPtr, const XsString *str)
{
	uint64_t tmp = 0;
	int count = 0;
	if (!thisPtr || !str || !str->m_data)
		return;
	int result = sscanf(str->m_data, "%" PRINTF_INT64_MODIFIER "x%n", &tmp, &count);
	if (result == 1)
		thisPtr->m_deviceId = tmp;
}

/*! \brief Get a string with a readable representation of this device ID. Either full or as a type
	\param makeType Boolean whether the deviceid is changed to a type instead of the full deviceid
	\param str The string to write to
*/
void XsDeviceId_toDeviceTypeString(const XsDeviceId* thisPtr, XsString* str, int makeType)
{
	XsDeviceId deviceType = XSDEVICEID_INITIALIZER;
	if (makeType)
		XsDeviceId_deviceType(thisPtr, 1, &deviceType);
	else
		deviceType = *thisPtr;
	char device[50];
	XsSize n;
	if (thisPtr->m_hardwareVersion == 0)
		n = (XsSize) (ptrdiff_t) sprintf(device, "%s_%08X.%08X", deviceType.m_productCode, (uint32_t)deviceType.m_deviceId, thisPtr->m_productVariant);
	else
		n = (XsSize) (ptrdiff_t) sprintf(device, "%s_%08X.%08X.%d_%d", deviceType.m_productCode, (uint32_t)deviceType.m_deviceId, thisPtr->m_productVariant, (uint8_t)((thisPtr->m_hardwareVersion & 0xFF00) >> 8), (uint8_t)(thisPtr->m_hardwareVersion & 0xFF));
	XsString_assign(str, n, device);
}

/*! \brief Read a device ID from the supplied string.
	\param str The string to interpret
*/
void XsDeviceId_fromDeviceTypeString(XsDeviceId* thisPtr, const XsString *str)
{
	uint32_t id = 0;
	int hwRevH = 0, hwRevL = 0;
	uint32_t variant;
	char productCode[24];
	if (!thisPtr || !str || !str->m_data)
		return;
	int result = sscanf(str->m_data, "%24[^_]_%08X.%08X.%d_%d", productCode, &id, &variant, &hwRevH, &hwRevL);
	if (result == 5 || result == 3)
	{
		thisPtr->m_deviceId = id;
		thisPtr->m_hardwareVersion = (uint16_t)((uint16_t)hwRevH << 8) + (uint16_t)hwRevL;
		thisPtr->m_productVariant = variant;
		strcpy(thisPtr->m_productCode, productCode);
	}
}

/*!	\brief Test if the device ID is a valid id (not 0).
*/
int XsDeviceId_isValid(const struct XsDeviceId* thisPtr)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return thisPtr->m_deviceId != 0;
	else
		return thisPtr->m_productCode[0] != 0 && thisPtr->m_deviceId != 0;
}

/*! \brief Swap the contents of \a a with those of \a b
*/
void XsDeviceId_swap(XsDeviceId* a, XsDeviceId* b)
{
	XsDeviceId tmp = *a;
	*a = *b;
	*b = tmp;
}

/*! \brief Returns true if this is equal to \a other or this is a type-specifier that matches \a other
	\param other The deviceid to compare this deviceid to
	\return True if the deviceids are equal or the type-specifier matches
*/
int XsDeviceId_contains(XsDeviceId const* thisPtr, XsDeviceId const* other)
{
	if (thisPtr == other)
		return 1;
	if (thisPtr->m_deviceId == other->m_deviceId)
		return 1;
	if (thisPtr->m_deviceId & XS_DID_ID_MASK) // NOTE: This produces incorrect results for device ids ending with 0000. See MVN-3876
		return 0;
	XsDeviceId typeThis, typeOther;
	XsDeviceId_deviceType(thisPtr, 1, &typeThis);
	XsDeviceId_deviceType(other, 1, &typeOther);
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return typeThis.m_deviceId == typeOther.m_deviceId;
	else
	{
		if (strcmp(typeThis.m_productCode, typeOther.m_productCode) == 0)
			return 1;
	}

	return 0;
}

/*! \brief Returns true if the ID is just a device type, not an actual device ID
*/
int XsDeviceId_isType(XsDeviceId const* thisPtr)
{
	// true when we have a valid type (not a broadcast) and a 0 ID
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
		return	!(thisPtr->m_deviceId & XS_DID_ID_MASK);
	else
		return	!(thisPtr->m_deviceId & ~XS_DID64_BIT);
}

/*! \brief Returns the name of the type of device identified by this id
*/
void XsDeviceId_typeName(XsDeviceId const* thisPtr, XsString* str)
{
	if (!str)
		return;

	if (!thisPtr)
	{
		XsString_assignCharArray(str, "invalid");
		return;
	}

	if (XsDeviceId_isAwinda2Station(thisPtr))
		XsString_assignCharArray(str, "Awinda Station v2");
	else if(XsDeviceId_isAwinda2Dongle(thisPtr))
		XsString_assignCharArray(str, "Awinda Dongle v2");
	else if(XsDeviceId_isAwinda2Oem(thisPtr))
		XsString_assignCharArray(str, "Awinda OEM v2");
	else if (XsDeviceId_isMtw2(thisPtr))
		XsString_assignCharArray(str, "MTw2");
	else if (XsDeviceId_isMtx2(thisPtr))
		XsString_assignCharArray(str, "MTx2");
	else if (XsDeviceId_isBodyPack(thisPtr))
		XsString_assignCharArray(str, "Bodypack");
	else if (XsDeviceId_isSyncStation2(thisPtr))
		XsString_assignCharArray(str, "Sync Station v2");
	else if (XsDeviceId_isMti6X0(thisPtr))
	{
		if (XsDeviceId_isImu(thisPtr))
			XsString_assignCharArray(str, "MTi-610");
		else if (XsDeviceId_isVru(thisPtr))
			XsString_assignCharArray(str, "MTi-620");
		else if (XsDeviceId_isAhrs(thisPtr))
			XsString_assignCharArray(str, "MTi-630");
		else if (XsDeviceId_isGnss(thisPtr))
			XsString_assignCharArray(str, "MTi-670");
	}
	else if (XsDeviceId_isMtMk4_1(thisPtr))
		XsString_assignCharArray(str, "MTi-1");
	else if (XsDeviceId_isMtMk4_2(thisPtr))
		XsString_assignCharArray(str, "MTi-2");
	else if (XsDeviceId_isMtMk4_3(thisPtr))
		XsString_assignCharArray(str, "MTi-3");
	else if (XsDeviceId_isMtMk4_7(thisPtr))
		XsString_assignCharArray(str, "MTi-7");
	else if (XsDeviceId_isMtMk4_10(thisPtr))
		XsString_assignCharArray(str, "MTi-10");
	else if (XsDeviceId_isMtMk4_20(thisPtr))
		XsString_assignCharArray(str, "MTi-20");
	else if (XsDeviceId_isMtMk4_30(thisPtr))
		XsString_assignCharArray(str, "MTi-30");
	else if (XsDeviceId_isMtMk4_100(thisPtr))
		XsString_assignCharArray(str, "MTi-100");
	else if (XsDeviceId_isMtMk4_200(thisPtr))
		XsString_assignCharArray(str, "MTi-200");
	else if (XsDeviceId_isMtMk4_300(thisPtr))
		XsString_assignCharArray(str, "MTi-300");
	else if ( XsDeviceId_isMtMk4_400(thisPtr))
		XsString_assignCharArray(str, "MTi-400");
	else if (XsDeviceId_isMtMk4_500(thisPtr))
		XsString_assignCharArray(str, "MTi-500");
	else if (XsDeviceId_isMtMk4_710(thisPtr))
		XsString_assignCharArray(str, "MTi-G-710");
	else if (XsDeviceId_isMtMk4_700(thisPtr))
		XsString_assignCharArray(str, "MTi-G-700");
	else if (XsDeviceId_isMtMk4_800(thisPtr))
		XsString_assignCharArray(str, "MTi-G-800");
	else if (XsDeviceId_isMtMk4_900(thisPtr))
		XsString_assignCharArray(str, "MTi-G-900");
	else
		XsString_assignCharArray(str, "Unknown");
}

/*! \brief Returns the type of device identified by this id
*/
void XsDeviceId_type(struct XsDeviceId const* thisPtr, struct XsDeviceId* type)
{
	if (!type)
		return;
	if (!XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		type->m_deviceId = XS_DID64_BIT;
		XsString typeName = XsString_INITIALIZER;
		XsDeviceId_typeName(thisPtr, &typeName);
		strcpy(type->m_productCode, typeName.m_data);
	}
	else
		type->m_deviceId = thisPtr->m_deviceId & XS_DID_FULLTYPE_MASK;
}

/*! \brief Returns the device type identified by this id (eg 10, 300 and Awinda2 Master)
*/
void XsDeviceId_deviceType(struct XsDeviceId const* thisPtr, int detailed, struct XsDeviceId* type)
{
	if (!type)
		return;
	if (!XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		if (XsDeviceId_isMti6X0(thisPtr))
		{
			type->m_deviceId = XS_DID64_BIT;
			strncpy(type->m_productCode, thisPtr->m_productCode, 7);
			type->m_productCode[7] = 0;
			if (detailed)
			{
				type->m_hardwareVersion = thisPtr->m_hardwareVersion;
				type->m_productVariant = thisPtr->m_productVariant;
			}
			else
			{
				type->m_hardwareVersion = 0;
				type->m_productVariant = 0;
			}
		}
	}
	else
	{
		XsDeviceId_deviceTypeMask(thisPtr, detailed, type);
		type->m_deviceId = thisPtr->m_deviceId & type->m_deviceId;
		strcpy(type->m_productCode, thisPtr->m_productCode);
	}
}

/*! \brief Returns the mask which can be used to get the detailed device type (eg 10, 300 and Awinda2 Master)
*/
void XsDeviceId_deviceTypeMask(struct XsDeviceId const* thisPtr, int detailed, struct XsDeviceId* type)
{
	if (XsDeviceId_isLegacyDeviceId(thisPtr))
	{
		if (XsDeviceId_isMtMk4_X(thisPtr))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK | XS_DID_TYPEL_MASK) : 0));
		else if (XsDeviceId_isMtMk4(thisPtr))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK | XS_DID_TYPEL_MK5) : 0));
		else if (XsDeviceId_isAwindaX(thisPtr))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK) : 0));
		else if (XsDeviceId_isSyncStationX(thisPtr))
			type->m_deviceId = (XS_DID_TYPEH_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK) : 0));
		else if (XsDeviceId_isMtw(thisPtr) || XsDeviceId_isMtx(thisPtr))
			type->m_deviceId = (XS_DID_TYPE_MASK | (detailed ? (XS_DID_GPH_MASK | XS_DID_GPL_MASK) : 0));
		else if (thisPtr->m_deviceId == XS_DID_ABMCLOCKMASTER)
			type->m_deviceId = XS_DID_ABMCLOCKMASTER;
		else
			type->m_deviceId = XS_DID_TYPEH_MASK;
	}
	else
		type->m_deviceId = XS_DID64_BIT;
}

//============================================================================================================
//============================================================================================================
//==== Deprecated methods follow                                                                         =====
//============================================================================================================
//============================================================================================================

/*! \brief Test if this device ID represents an MTMk4.
	\returns True if it is an MTMk4.
	\deprecated If the purpose is to check for a Mk4, Mk5 or 1-series device use: isMti() || isMtig(). If the purpose is to detect actual Mk4 (10,100,7x0) use isMtMark4()
*/
int XsDeviceId_isMtMk4(const struct XsDeviceId* thisPtr)
{
	return (XsDeviceId_isMtMk4_X(thisPtr) ||
		XsDeviceId_isMtMk4_X0(thisPtr) ||
		XsDeviceId_isMtMk4_X00(thisPtr));
}

/*! \brief Test if this device ID represents an MTMk4 1 series.
	\returns True if it is an MTMk4 1 series.
	\deprecated Use isMtiX()
*/
int XsDeviceId_isMtMk4_X(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 1.
	\returns True if it is an MTMk4 1.
	\deprecated Use isMtiX() and isImu()
*/
int XsDeviceId_isMtMk4_1(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX(thisPtr) && XsDeviceId_isImu(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 2.
	\returns True if it is an MTMk4 2.
	\deprecated Use isMtiX() and isVru()
*/
int XsDeviceId_isMtMk4_2(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX(thisPtr) && XsDeviceId_isVru(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 3.
	\returns True if it is an MTMk4 3.
	\deprecated Use isMtiX() and isAhrs()
*/
int XsDeviceId_isMtMk4_3(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX(thisPtr) && XsDeviceId_isAhrs(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 7.
	\returns True if it is an MTMk4 7.
	\deprecated Use isMtiX() and isGnss()
*/
int XsDeviceId_isMtMk4_7(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX(thisPtr) && XsDeviceId_isGnss(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 10 series.
	\returns True if it is an MTMk4 10 series.
	\deprecated Use isMtiX0()
*/
int XsDeviceId_isMtMk4_X0(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX0(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 10.
	\returns True if it is an MTMk4 10.
	\deprecated Use isMtiX0() and isImu()
*/
int XsDeviceId_isMtMk4_10(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX0(thisPtr) && XsDeviceId_isImu(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 20.
	\returns True if it is an MTMk4 20.
	\deprecated Use isMtiX0() and isVru()
*/
int XsDeviceId_isMtMk4_20(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX0(thisPtr) && XsDeviceId_isVru(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 30.
	\returns True if it is an MTMk4 30.
	\deprecated Use isMtiX0() and isAhrs()
*/
int XsDeviceId_isMtMk4_30(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX0(thisPtr) && XsDeviceId_isAhrs(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 100 series (including 700 and 710)
	\returns True if it is an MTMk4 100 series.
	\deprecated Use isMtiX00() || isMtig() to detect all 100's and 7x0's
*/
int XsDeviceId_isMtMk4_X00(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX00(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 100.
	\returns True if it is an MTMk4 100.
	\deprecated Use isMtiX00() and isImu()
*/
int XsDeviceId_isMtMk4_100(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX00(thisPtr) && XsDeviceId_isImu(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 200.
	\returns True if it is an MTMk4 200.
	\deprecated Use isMtiX00() and isVru()
*/
int XsDeviceId_isMtMk4_200(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX00(thisPtr) && XsDeviceId_isVru(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 300.
	\returns True if it is an MTMk4 300.
	\deprecated Use isMtiX00() and isAhrs()
*/
int XsDeviceId_isMtMk4_300(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtiX00(thisPtr) && XsDeviceId_isAhrs(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 400.
	\returns True if it is an MTMk4 400.
	\deprecated Will be removed
*/
int XsDeviceId_isMtMk4_400(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_400);
}

/*! \brief Test if this device ID represents an MTMk4 500.
	\returns True if it is an MTMk4 500.
	\deprecated Will be removed
*/
int XsDeviceId_isMtMk4_500(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_500);
}

/*! \brief Test if this device ID represents an MTMk4 600.
	\returns True if it is an MTMk4 600.
	\deprecated Will be removed
*/
int XsDeviceId_isMtMk4_600(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_600);
}

/*! \brief Test if this device ID represents an MTMk4 700.
	\returns True for all 700 and 710's (Mk4 and Mk5).
	\deprecated use isGnss() to check for Gnss capabilities,
				use isMtig() to check for either 700 or 710,
				use isMtigX00() or isMtigX10() to discern between 700 and 710,
				use isMtMark4() or isMtMark5() to discern between Mark 4 and Mark 5
*/
int XsDeviceId_isMtMk4_700(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtigX00(thisPtr) || XsDeviceId_isMtigX10(thisPtr);
}

/*! \brief Test if this device ID represents either an MTMk4 710 or Mk5 710.
	\returns True if it is either an MTMk4 710 or Mk5 710.
	\deprecated use isGnss() to check for Gnss capabilities, use isMtig() to check for either 700 or 710, use isMtigX00() or isMtigX10() to discern between 700 and 710
*/
int XsDeviceId_isMtMk4_710(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtigX10(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk4 800.
	\returns True if it is an MTMk4 800.
	\deprecated Will be removed
*/
int XsDeviceId_isMtMk4_800(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_800);
}

/*! \brief Test if this device ID represents an MTMk4 900.
	\returns True if it is an MTMk4 900.
	\deprecated Will be removed
*/
int XsDeviceId_isMtMk4_900(const struct XsDeviceId* thisPtr)
{
	return ((thisPtr->m_deviceId & XS_DID_MK4TYPE_MASK) == XS_DID_MK4TYPE_MT_900);
}

/*! \brief Test if this device ID represents an MTMk5.
	\returns True if it is an MTMk5.
	\deprecated Use isMtMark5()
*/
int XsDeviceId_isMtMk5(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMark5(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 10 series.
	\returns True if it is an MTMk5 10 series.
	\deprecated Use isMtMark5() && isMtiX0()
*/
int XsDeviceId_isMtMk5_X0(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_X0(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 10.
	\returns True if it is an MTMk5 10.
	\deprecated Use isMtMark5() && isMtiX0() && isImu()
*/
int XsDeviceId_isMtMk5_10(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_10(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 20.
	\returns True if it is an MTMk5 20.
	\deprecated Use isMtMark5() && isMtiX0() && isVru()
*/
int XsDeviceId_isMtMk5_20(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_20(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 30.
	\returns True if it is an MTMk5 30.
	\deprecated Use isMtMark5() && isMtiX0() && isAhrs()
*/
int XsDeviceId_isMtMk5_30(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_30(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 100 series.
	\returns True if it is an MTMk5 100 series.
	\deprecated Use isMtMark5() && (isMtiX00() || isMtig())
*/
int XsDeviceId_isMtMk5_X00(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_X00(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 100.
	\returns True if it is an MTMk5 100.
	\deprecated Use isMtMark5() && isMtiX00() && isImu()
*/
int XsDeviceId_isMtMk5_100(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_100(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 200.
	\returns True if it is an MTMk5 200.
	\deprecated Use isMtMark5() && isMtiX00() && isVru()
*/
int XsDeviceId_isMtMk5_200(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_200(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 300.
	\returns True if it is an MTMk5 300.
	\deprecated Use isMtMark5() && isMtiX00() && isAhrs()
*/
int XsDeviceId_isMtMk5_300(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_300(thisPtr);
}

/*! \brief Test if this device ID represents an MTMk5 710.
	\returns True if it is an MTMk5 710.
	\deprecated Use (isMtMark5() && isMtigX10())
*/
int XsDeviceId_isMtMk5_710(const struct XsDeviceId* thisPtr)
{
	return XsDeviceId_isMtMk5(thisPtr) && XsDeviceId_isMtMk4_710(thisPtr);
}

/*! @} */
