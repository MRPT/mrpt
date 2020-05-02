
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

#ifndef XSDEVICECAPABILITIES_H
#define XSDEVICECAPABILITIES_H

#include "xstypesconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!	\addtogroup enums Global enumerations
	@{
*/
/*!	\brief Device capability flags. */
enum XsDeviceCapability
{
	XDC_Invalid		= 0x00000000,		//!<	Indicates the \a XsDeviceCapabilities structure is invalid.
	XDC_Acc			= 0x00000001,		//!<	Device has an operational accelerometer.
	XDC_Gyr			= 0x00000002,		//!<	Device has an operational gyroscope.
	XDC_Mag			= 0x00000004,		//!<	Device has an operational magnetometer.
	XDC_Baro		= 0x00000008,		//!<	Device has an operational barometer
	XDC_Gnss		= 0x00000010,		//!<	Device has an operational GNSS receiver.
	XDC_Imu			= 0x00000020,		//!<	Device is an IMU.
	XDC_Vru			= 0x00000040,		//!<	Device is a VRU.
	XDC_Ahrs		= 0x00000080,		//!<	Device is an AHRS.
	XDC_GnssIns		= 0x00000100,		//!<	Device is a GNSS/INS.
};
/*! @} */

typedef enum XsDeviceCapability XsDeviceCapability;

struct XsDeviceCapabilities;

XSTYPES_DLL_API int XsDeviceCapabilities_isValid(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_hasAccelerometer(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_hasGyroscope(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_hasMagnetometer(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_hasBarometer(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_hasGnss(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_isImu(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_isVru(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_isAhrs(const struct XsDeviceCapabilities* thisPtr);
XSTYPES_DLL_API int XsDeviceCapabilities_isGnssIns(const struct XsDeviceCapabilities* thisPtr);

#ifdef __cplusplus
}
#endif

struct XsDeviceCapabilities
{
#ifdef __cplusplus
public:
	//!	\brief	Constructor to create an XsDeviceCapabilities instance from a raw flags field.
	inline explicit XsDeviceCapabilities(uint32_t flags = XDC_Invalid)
		: m_flags(flags)
	{
	}

	//!	\brief	Copy constructor.
	inline XsDeviceCapabilities(const XsDeviceCapabilities& other)
		: m_flags(other.m_flags)
	{
	}

	//!	\brief	Copy assignment.
	inline const XsDeviceCapabilities& operator=(const XsDeviceCapabilities& other)
	{
		m_flags = other.m_flags;
		return *this;
	}

	//!	\copydoc XsDeviceCapabilities_hasAccelerometer(const struct XsDeviceCapabilities*)
	inline bool hasAccelerometer() const
	{
		return XsDeviceCapabilities_hasAccelerometer(this) != 0;
	}

	//!	\copydoc XsDeviceCapabilities_hasGyroscope(const struct XsDeviceCapabilities*)
	inline bool hasGyroscope() const
	{
		return XsDeviceCapabilities_hasGyroscope(this) != 0;
	}

	//!	\copydoc XsDeviceCapabilities_hasMagnetometer(const struct XsDeviceCapabilities*)
	inline bool hasMagnetometer() const
	{
		return XsDeviceCapabilities_hasMagnetometer(this) != 0;
	}

	//!	\copydoc XsDeviceCapabilities_hasBarometer(const struct XsDeviceCapabilities*)
	inline bool hasBarometer() const
	{
		return XsDeviceCapabilities_hasBarometer(this) != 0;
	}

	//!	\copydoc XsDeviceCapabilities_hasGnss(const struct XsDeviceCapabilities*)
	inline bool hasGnss() const
	{
		return XsDeviceCapabilities_hasGnss(this) != 0;
	}

	//!	\copydoc XsDeviceCapabilities_isImu(const struct XsDeviceCapabilities*)
	inline bool isImu() const
	{
		return XsDeviceCapabilities_isImu(this) != 0;
	}

	//!	\copydoc XsDeviceCapabilities_isVru(const struct XsDeviceCapabilities*)
	inline bool isVru() const
	{
		return XsDeviceCapabilities_isVru(this) != 0;
	}

	//!	\copydoc XsDeviceCapabilities_isAhrs(const struct XsDeviceCapabilities*)
	inline bool isAhrs() const
	{
		return XsDeviceCapabilities_isAhrs(this) != 0;
	}

	//!	\copydoc XsDeviceCapabilities_isGnssIns(const struct XsDeviceCapabilities*)
	inline bool isGnssIns() const
	{
		return XsDeviceCapabilities_isGnssIns(this) != 0;
	}
private:
#endif

	uint32_t m_flags;
};

typedef struct XsDeviceCapabilities XsDeviceCapabilities;

#endif
