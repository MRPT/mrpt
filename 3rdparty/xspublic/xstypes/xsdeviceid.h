
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

#ifndef XSDEVICEID_H
#define XSDEVICEID_H

#include "xstypesconfig.h"
#include "pstdint.h"
#include "xsstring.h"
#ifdef __cplusplus
extern "C" {
#endif

#define XSDEVICEID_PRODUCT_CODE_LEN		24
#define XSDEVICEID_PRODUCT_CODE_INIT	"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"

#ifndef __cplusplus
#define XSDEVICEID_INITIALIZER	{ 0, XSDEVICEID_PRODUCT_CODE_INIT, 0, 0 }
#endif

struct XsDeviceId;

XSTYPES_DLL_API int XsDeviceId_isLegacyDeviceId(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API uint64_t XsDeviceId_legacyBit(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtiX(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtiX0(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtiX00(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtigX00(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtigX10(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMti6X0(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtw(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtw2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtx(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtx2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isBodyPack(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isWirelessMaster(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaX(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaXStation(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaXDongle(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwindaXOem(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda2Station(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda2Dongle(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAwinda2Oem(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isSyncStationX(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isSyncStation2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isHilDevice(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isGlove(struct XsDeviceId const* thisPtr);

XSTYPES_DLL_API int XsDeviceId_isImu(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isVru(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isAhrs(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isGnss(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isContainerDevice(struct XsDeviceId const* thisPtr);

XSTYPES_DLL_API int XsDeviceId_isMt(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMti(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtig(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMark4(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMark5(struct XsDeviceId const* thisPtr);

//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------

XSTYPES_DLL_API void XsDeviceId_toString(struct XsDeviceId const* thisPtr, XsString* str);
XSTYPES_DLL_API void XsDeviceId_fromString(struct XsDeviceId* thisPtr, XsString const* str);
XSTYPES_DLL_API void XsDeviceId_toDeviceTypeString(struct XsDeviceId const* thisPtr, XsString* str, int makeType);
XSTYPES_DLL_API void XsDeviceId_fromDeviceTypeString(struct XsDeviceId* thisPtr, XsString const* str);
XSTYPES_DLL_API int XsDeviceId_isValid(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API void XsDeviceId_swap(struct XsDeviceId* a, struct XsDeviceId* b);
XSTYPES_DLL_API int XsDeviceId_contains(struct XsDeviceId const* a, struct XsDeviceId const* b);
XSTYPES_DLL_API int XsDeviceId_isType(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API void XsDeviceId_typeName(struct XsDeviceId const* thisPtr, XsString* str);
XSTYPES_DLL_API void XsDeviceId_type(struct XsDeviceId const* thisPtr, struct XsDeviceId* type);
XSTYPES_DLL_API void XsDeviceId_deviceType(struct XsDeviceId const* thisPtr, int detailed, struct XsDeviceId* type);
XSTYPES_DLL_API void XsDeviceId_deviceTypeMask(struct XsDeviceId const* thisPtr, int detailed, struct XsDeviceId* type);

//============================================================================================================
//==== Deprecated methods follow                                                                         =====
//============================================================================================================
XSTYPES_DLL_API int XsDeviceId_isMtMk4(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_X(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_1(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_2(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_3(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_7(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_X0(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_10(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_20(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_30(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_X00(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_100(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_200(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_300(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_400(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_500(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_600(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_700(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_710(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_800(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk4_900(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_X0(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_10(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_20(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_30(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_X00(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_100(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_200(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_300(struct XsDeviceId const* thisPtr);
XSTYPES_DLL_API int XsDeviceId_isMtMk5_710(struct XsDeviceId const* thisPtr);
//============================================================================================================

#ifdef __cplusplus
} // extern "C"
#endif

struct XsDeviceId {
#ifdef __cplusplus
	/*! \brief Constructor that creates an XsDeviceId from the supplied \a productcode, \a hardwareVersion, \a productVariant and \a serialNumber*/
	inline XsDeviceId(const char* productCode, uint16_t hardwareVersion, uint32_t productVariant, uint64_t serialNumber)
		: m_deviceId(serialNumber)
		, m_hardwareVersion(hardwareVersion)
		, m_productVariant(productVariant)
	{
		memset(m_productCode, 0, sizeof(m_productCode));
		if (productCode)
			strcpy(m_productCode, productCode);
	}

	/*! \brief Constructor that creates an XsDeviceId from the supplied \a deviceId */
	inline XsDeviceId(uint64_t serialNumber = 0)
		: m_deviceId(serialNumber)
		, m_hardwareVersion(0)
		, m_productVariant(0)
	{
		memset(m_productCode, 0, sizeof(m_productCode));
	}

	/*! \brief Constructor that creates an XsDeviceId from the supplied XsDeviceId \a other */
	inline XsDeviceId(const XsDeviceId& other)
	{
		memcpy(static_cast<void*>(this), static_cast<void const*>(&other), sizeof(XsDeviceId));
	}
	/*! \brief Assign the \a other deviceId to this XsDeviceId */
	inline const XsDeviceId& operator=(const XsDeviceId& other)
	{
		if (this != &other)
			memcpy(static_cast<void*>(this), static_cast<void const*>(&other), sizeof(XsDeviceId));
		return *this;
	}

	/*! \brief \copybrief XsDeviceId_legacyBit(const struct XsDeviceId*) */
	static inline uint64_t legacyBit()
	{
		return XsDeviceId_legacyBit(nullptr);
	}
	/*! \brief \copybrief XsDeviceId_isLegacyDeviceId(const struct XsDeviceId*) */
	inline bool isLegacyDeviceId() const
	{
		return 0 != XsDeviceId_isLegacyDeviceId(this);
	}
	/*! \brief Returns the device serial number as an unsigned integer */
	inline uint64_t toInt() const
	{
		return m_deviceId;
	}
	/*! \brief Returns the product code */
	inline XsString productCode() const
	{
		XsString tmp(m_productCode);
		return tmp;
	}
	/*! \brief Returns the product variant */
	inline uint32_t productVariant() const
	{
		return m_productVariant;
	}
	/*! \brief Returns the product variant */
	inline uint16_t hardwareVersion() const
	{
		return m_hardwareVersion;
	}
	/*! \brief Returns the 32-bit device serial number, which may be 0 if the device has a 64-bit serial number */
	inline uint32_t legacyDeviceId() const
	{
		return static_cast<uint32_t>(m_deviceId);
	}
	/*! \brief \copybrief XsDeviceId_isMtiX(const struct XsDeviceId*) */
	inline bool isMtiX() const
	{
		return 0 != XsDeviceId_isMtiX(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtiX0(const struct XsDeviceId*) */
	inline bool isMtiX0() const
	{
		return 0 != XsDeviceId_isMtiX0(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtiX00(const struct XsDeviceId*) */
	inline bool isMtiX00() const
	{
		return 0 != XsDeviceId_isMtiX00(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtigX00(const struct XsDeviceId*) */
	inline bool isMtigX00() const
	{
		return 0 != XsDeviceId_isMtigX00(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtigX10(const struct XsDeviceId*) */
	inline bool isMtigX10() const
	{
		return 0 != XsDeviceId_isMtigX10(this);
	}
	/*! \brief \copybrief XsDeviceId_isMti6X0(const struct XsDeviceId*) */
	inline bool isMti6X0() const
	{
		return 0 != XsDeviceId_isMti6X0(this);
	}
	/*! \brief \copybrief XsDeviceId_isGlove(const struct XsDeviceId*) */
	inline bool isGlove () const
	{
		return 0 != XsDeviceId_isGlove(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtw(const struct XsDeviceId*) */
	inline bool isMtw() const
	{
		return 0 != XsDeviceId_isMtw(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtw2(const struct XsDeviceId*) */
	inline bool isMtw2() const
	{
		return 0 != XsDeviceId_isMtw2(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtx(const struct XsDeviceId*) */
	inline bool isMtx() const
	{
		return 0 != XsDeviceId_isMtx(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtx2(const struct XsDeviceId*) */
	inline bool isMtx2() const
	{
		return 0 != XsDeviceId_isMtx2(this);
	}
	/*! \brief \copybrief XsDeviceId_isBodyPack(const struct XsDeviceId*) */
	inline bool isBodyPack() const
	{
		return 0 != XsDeviceId_isBodyPack(this);
	}
	/*! \brief \copybrief XsDeviceId_isWirelessMaster(const struct XsDeviceId*) */
	inline bool isWirelessMaster() const
	{
		return 0 != XsDeviceId_isWirelessMaster(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaX(const struct XsDeviceId*) */
	inline bool isAwindaX() const
	{
		return 0 != XsDeviceId_isAwindaX(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaXStation(const struct XsDeviceId*) */
	inline bool isAwindaXStation() const
	{
		return 0 != XsDeviceId_isAwindaXStation(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaXDongle(const struct XsDeviceId*) */
	inline bool isAwindaXDongle() const
	{
		return 0 != XsDeviceId_isAwindaXDongle(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwindaXOem(const struct XsDeviceId*) */
	inline bool isAwindaXOem() const
	{
		return 0 != XsDeviceId_isAwindaXOem(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda2(const struct XsDeviceId*) */
	inline bool isAwinda2() const
	{
		return 0 != XsDeviceId_isAwinda2(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda2Station(const struct XsDeviceId*) */
	inline bool isAwinda2Station() const
	{
		return 0 != XsDeviceId_isAwinda2Station(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda2Dongle(const struct XsDeviceId*) */
	inline bool isAwinda2Dongle() const
	{
		return 0 != XsDeviceId_isAwinda2Dongle(this);
	}
	/*! \brief \copybrief XsDeviceId_isAwinda2Oem(const struct XsDeviceId*) */
	inline bool isAwinda2Oem() const
	{
		return 0 != XsDeviceId_isAwinda2Oem(this);
	}
	/*! \brief \copybrief XsDeviceId_isSyncStationX(const struct XsDeviceId*) */
	inline bool isSyncStationX() const
	{
		return 0 != XsDeviceId_isSyncStationX(this);
	}
	/*! \brief \copybrief XsDeviceId_isSyncStation2(const struct XsDeviceId*) */
	inline bool isSyncStation2() const
	{
		return 0 != XsDeviceId_isSyncStation2(this);
	}
	/*! \brief \copybrief XsDeviceId_isHilDevice(const struct XsDeviceId*) */
	inline bool isHilDevice() const
	{
		return 0 != XsDeviceId_isHilDevice(this);
	}
	/*! \brief \copybrief XsDeviceId_isImu(const struct XsDeviceId*) */
	inline bool isImu() const
	{
		return 0 != XsDeviceId_isImu(this);
	}
	/*! \brief \copybrief XsDeviceId_isVru(const struct XsDeviceId*) */
	inline bool isVru() const
	{
		return 0 != XsDeviceId_isVru(this);
	}
	/*! \brief \copybrief XsDeviceId_isAhrs(const struct XsDeviceId*) */
	inline bool isAhrs() const
	{
		return 0 != XsDeviceId_isAhrs(this);
	}
	/*! \brief \copybrief XsDeviceId_isGnss(const struct XsDeviceId*) */
	inline bool isGnss() const
	{
		return 0 != XsDeviceId_isGnss(this);
	}
	/*! \brief \copybrief XsDeviceId_isContainerDevice(const struct XsDeviceId*) */
	inline bool isContainerDevice() const
	{
		return 0 != XsDeviceId_isContainerDevice(this);
	}
	/*! \brief \copybrief XsDeviceId_isMt(const struct XsDeviceId*) */
	inline bool isMt() const
	{
		return 0 != XsDeviceId_isMt(this);
	}
	/*! \brief \copybrief XsDeviceId_isMti(const struct XsDeviceId*) */
	inline bool isMti() const
	{
		return 0 != XsDeviceId_isMti(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtig(const struct XsDeviceId*) */
	inline bool isMtig() const
	{
		return 0 != XsDeviceId_isMtig(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMark4(const struct XsDeviceId*) */
	inline bool isMtMark4() const
	{
		return 0 != XsDeviceId_isMtMark4(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMark5(const struct XsDeviceId*) */
	inline bool isMtMark5() const
	{
		return 0 != XsDeviceId_isMtMark5(this);
	}
	/*! \brief \copybrief XsDeviceId_toString(const XsDeviceId* thisPtr, XsString* str) */
	inline XsString toString() const
	{
		XsString tmp;
		XsDeviceId_toString(this, &tmp);
		return tmp;
	}
	/*! \copydoc XsDeviceId_fromString(XsDeviceId* thisPtr, const XsString* str) */
	inline void fromString(const XsString &str)
	{
		XsDeviceId_fromString(this, &str);
	}

	/*! \copybrief XsDeviceId_toDeviceTypeString(const XsDeviceId* thisPtr, XsString* str, int makeType) 
		\param makeType Boolean whether the deviceid is made a devicetype or is printed fully in the output string
		\return The string with the device type or device id information in string format
	*/
	inline XsString toDeviceTypeString(bool makeType = true) const
	{
		XsString tmp;
		XsDeviceId_toDeviceTypeString(this, &tmp, makeType ? 1 : 0);
		return tmp;
	}

	/*! \copydoc XsDeviceId_fromDeviceTypeString(XsDeviceId* thisPtr, const XsString* str)
	*/
	inline void fromDeviceTypeString(const XsString &str)
	{
		XsDeviceId_fromDeviceTypeString(this, &str);
	}

	/*! \brief \copybrief XsDeviceId_isValid(const struct XsDeviceId*) */
	inline bool isValid() const
	{
		return 0 != XsDeviceId_isValid(this);
	}
	/*! \copydoc XsDeviceId_contains(XsDeviceId const*, XsDeviceId const*) */
	inline bool contains(const XsDeviceId& other) const
	{
		return 0 != XsDeviceId_contains(this, &other);
	}
	/*! \brief Returns true if the ID is just a device type, not an actual device ID */
	inline bool isType() const
	{
		return 0 != XsDeviceId_isType(this);
	}
	/*! \brief Returns the name of the type of device identified by this id */
	inline XsString typeName() const
	{
		XsString rv;
		XsDeviceId_typeName(this, &rv);
		return rv;
	}
	/*! \brief Returns the type of device identified by this id */
	inline XsDeviceId type() const
	{
		XsDeviceId xtype;
		XsDeviceId_type(this, &xtype);
		return xtype;
	}
	/*! \brief Returns the (detailed) device type of this id
	\param detailed Boolean whether detailed information is returned
	\return The requested device type
	*/
	inline XsDeviceId deviceType(bool detailed = true) const
	{
		XsDeviceId xtype;
		XsDeviceId_deviceType(this, detailed ? 1 : 0, &xtype);
		return xtype;
	}
	/*! \brief Returns the detailed device type mask of this id
	\param detailed Boolean whether detailed information is returned
	\return The requested device type mask
	*/
	inline XsDeviceId deviceTypeMask(bool detailed = true) const
	{
		XsDeviceId xtype;
		XsDeviceId_deviceTypeMask(this, detailed ? 1 : 0, &xtype);
		return xtype;
	}

	/*! \brief Returns true if the \a other deviceId matches this deviceId */
	inline bool operator==(const XsDeviceId& other) const
	{
		if (isLegacyDeviceId() || other.isLegacyDeviceId())
			return toInt() == other.toInt();
		else
		{
			return (toInt() == other.toInt() &&
					m_productVariant == other.m_productVariant &&
					m_hardwareVersion == other.m_hardwareVersion &&
					(strcmp(m_productCode, other.m_productCode) == 0)
					);
		}
	}
	/*! \brief Returns true if the \a other deviceId does not match this deviceId */
	inline bool operator!=(const XsDeviceId& other) const { return !(*this == other); }
	/*! \brief Returns true if this deviceId is less than the \a other deviceId */
	inline bool operator<(const XsDeviceId& other) const
	{
		if (isLegacyDeviceId() || other.isLegacyDeviceId())
			return toInt() < other.toInt();
		else
		{
			int pdiff = strcmp(m_productCode, other.m_productCode);
			if (pdiff < 0)
				return true;
			else if (pdiff > 0)
				return false;
			else
			{
				if (m_hardwareVersion < other.m_hardwareVersion)
					return true;
				else if (m_hardwareVersion > other.m_hardwareVersion)
					return false;
				else
				{
					if (m_productVariant < other.m_productVariant)
						return true;
					else if (m_productVariant > other.m_productVariant)
						return false;
					else
					{
						if (toInt() < other.toInt())
							return true;
						else if (toInt() > other.toInt())
							return false;
						else
							return false;
					}
				}
			}
		}
	}
	/*! \brief Returns true if this deviceId is less or equal to the \a other deviceId */
	inline bool operator<=(const XsDeviceId& other) const
	{
		if (isLegacyDeviceId())
			return toInt() <= other.toInt();
		else
		{
			int pdiff = strcmp(m_productCode, other.m_productCode);
			if (pdiff > 0)
				return false;
			else
			{
				if (m_hardwareVersion > other.m_hardwareVersion)
					return false;
				else
				{
					if (m_productVariant > other.m_productVariant)
						return false;
					else
					{
						if (toInt() > other.toInt())
							return false;
						else
							return true;
					}
				}
			}
		}
	}
	/*! \brief Returns true if this deviceId is larger than the \a other deviceId */
	inline bool operator>(const XsDeviceId& other) const
	{
		if (isLegacyDeviceId())
			return toInt() > other.toInt();
		else
		{
			int pdiff = strcmp(m_productCode, other.m_productCode);
			if (pdiff > 0)
				return true;
			else if (pdiff < 0)
				return false;
			else
			{
				if (m_hardwareVersion > other.m_hardwareVersion)
					return true;
				else if (m_hardwareVersion < other.m_hardwareVersion)
					return false;
				else
				{
					if (m_productVariant > other.m_productVariant)
						return true;
					else if (m_productVariant < other.m_productVariant)
						return false;
					else
					{
						if (toInt() > other.toInt())
							return true;
						else if (toInt() < other.toInt())
							return false;
						else
							return false;
					}
				}
			}
		}
	}
	/*! \brief Returns true if this deviceId is larger or equal to the \a other deviceId */
	inline bool operator>=(const XsDeviceId& other) const
	{
		if (isLegacyDeviceId())
			return toInt() >= other.toInt();
		else
		{
			int pdiff = strcmp(m_productCode, other.m_productCode);
			if (pdiff < 0)
				return false;
			else
			{
				if (m_hardwareVersion < other.m_hardwareVersion)
					return false;
				else
				{
					if (m_productVariant < other.m_productVariant)
						return false;
					else
					{
						if (toInt() < other.toInt())
							return false;
						else
							return true;
					}
				}
			}
		}
	}


//============================================================================================================
//============================================================================================================
//==== Deprecated methods follow                                                                         =====
//============================================================================================================
//============================================================================================================

	/*! \brief \copybrief XsDeviceId_isMtMk4(const struct XsDeviceId*) */
	inline bool isMtMk4() const
	{
		return 0 != XsDeviceId_isMtMk4(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_X(const struct XsDeviceId*) */
	inline bool isMtMk4_X() const
	{
		return 0 != XsDeviceId_isMtMk4_X(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_1(const struct XsDeviceId*) */
	inline bool isMtMk4_1() const
	{
		return 0 != XsDeviceId_isMtMk4_1(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_2(const struct XsDeviceId*) */
	inline bool isMtMk4_2() const
	{
		return 0 != XsDeviceId_isMtMk4_2(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_3(const struct XsDeviceId*) */
	inline bool isMtMk4_3() const
	{
		return 0 != XsDeviceId_isMtMk4_3(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_7(const struct XsDeviceId*) */
	inline bool isMtMk4_7() const
	{
		return 0 != XsDeviceId_isMtMk4_7(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_X0(const struct XsDeviceId*) */
	inline bool isMtMk4_X0() const
	{
		return 0 != XsDeviceId_isMtMk4_X0(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_10(const struct XsDeviceId*) */
	inline bool isMtMk4_10() const
	{
		return 0 != XsDeviceId_isMtMk4_10(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_20(const struct XsDeviceId*) */
	inline bool isMtMk4_20() const
	{
		return 0 != XsDeviceId_isMtMk4_20(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_30(const struct XsDeviceId*) */
	inline bool isMtMk4_30() const
	{
		return 0 != XsDeviceId_isMtMk4_30(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_X00(const struct XsDeviceId*) */
	inline bool isMtMk4_X00() const
	{
		return 0 != XsDeviceId_isMtMk4_X00(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_100(const struct XsDeviceId*) */
	inline bool isMtMk4_100() const
	{
		return 0 != XsDeviceId_isMtMk4_100(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_200(const struct XsDeviceId*) */
	inline bool isMtMk4_200() const
	{
		return 0 != XsDeviceId_isMtMk4_200(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_300(const struct XsDeviceId*) */
	inline bool isMtMk4_300() const
	{
		return 0 != XsDeviceId_isMtMk4_300(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_400(const struct XsDeviceId*) */
	inline bool isMtMk4_400() const
	{
		return 0 != XsDeviceId_isMtMk4_400(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_500(const struct XsDeviceId*) */
	inline bool isMtMk4_500() const
	{
		return 0 != XsDeviceId_isMtMk4_500(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_600(const struct XsDeviceId*) */
	inline bool isMtMk4_600() const
	{
		return 0 != XsDeviceId_isMtMk4_600(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_700(const struct XsDeviceId*) */
	inline bool isMtMk4_700() const
	{
		return 0 != XsDeviceId_isMtMk4_700(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_710(const struct XsDeviceId*) */
	inline bool isMtMk4_710() const
	{
		return 0 != XsDeviceId_isMtMk4_710(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_800(const struct XsDeviceId*) */
	inline bool isMtMk4_800() const
	{
		return 0 != XsDeviceId_isMtMk4_800(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk4_900(const struct XsDeviceId*) */
	inline bool isMtMk4_900() const
	{
		return 0 != XsDeviceId_isMtMk4_900(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5(const struct XsDeviceId*) */
	inline bool isMtMk5() const
	{
		return 0 != XsDeviceId_isMtMk5(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_X0(const struct XsDeviceId*) */
	inline bool isMtMk5_X0() const
	{
		return 0 != XsDeviceId_isMtMk5_X0(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_10(const struct XsDeviceId*) */
	inline bool isMtMk5_10() const
	{
		return 0 != XsDeviceId_isMtMk5_10(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_20(const struct XsDeviceId*) */
	inline bool isMtMk5_20() const
	{
		return 0 != XsDeviceId_isMtMk5_20(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_30(const struct XsDeviceId*) */
	inline bool isMtMk5_30() const
	{
		return 0 != XsDeviceId_isMtMk5_30(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_X00(const struct XsDeviceId*) */
	inline bool isMtMk5_X00() const
	{
		return 0 != XsDeviceId_isMtMk5_X00(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_100(const struct XsDeviceId*) */
	inline bool isMtMk5_100() const
	{
		return 0 != XsDeviceId_isMtMk5_100(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_200(const struct XsDeviceId*) */
	inline bool isMtMk5_200() const
	{
		return 0 != XsDeviceId_isMtMk5_200(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_300(const struct XsDeviceId*) */
	inline bool isMtMk5_300() const
	{
		return 0 != XsDeviceId_isMtMk5_300(this);
	}
	/*! \brief \copybrief XsDeviceId_isMtMk5_710(const struct XsDeviceId*) */
	inline bool isMtMk5_710() const
	{
		return 0 != XsDeviceId_isMtMk5_710(this);
	}

//============================================================================================================

private:
#endif
	uint64_t m_deviceId; //!< The serialnumber of a device
	char m_productCode[24]; //!< The productcode of a device
	uint16_t m_hardwareVersion; //!< The hardware version of a device
	uint32_t m_productVariant;	//!< The product variant of a device
};

typedef struct XsDeviceId XsDeviceId;

#if defined(__cplusplus) && !defined(XSENS_NO_STL)
namespace std
{
	template<typename _CharT, typename _Traits>
	basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsDeviceId const& xd)
	{
		return (o << xd.toString());
	}
}

inline XsString& operator<<(XsString& o, XsDeviceId const& xd)
{
	o.append(xd.toString());
	return o;
}

#endif

#endif
