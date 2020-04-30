
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

#include "xsdevicecapabilities.h"

/*! \class XsDeviceCapabilities
	\brief Describes what a device is capable of.
*/

/*! \addtogroup cinterface C Interface
@{
*/

/*!	\brief	Test if the given \a XsDeviceCapabilities is valid.
	\return True if valid.
*/
int XsDeviceCapabilities_isValid(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags != XDC_Invalid);
}

/*!	\brief	Test if the device has an operational accelerometer.
	\return True if the Accelerometer flag is set
*/
int XsDeviceCapabilities_hasAccelerometer(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_Acc) > 0;
}

/*!	\brief	Test if the device has an operational gyroscope.
	\return True if the Gyroscope flag is set
*/
int XsDeviceCapabilities_hasGyroscope(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_Gyr) > 0;
}

/*!	\brief	Test if the device has an operational magnetometer.
	\return True if the Magnetometer flag is set
*/
int XsDeviceCapabilities_hasMagnetometer(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_Mag) > 0;
}

/*!	\brief	Test if the device has an operational barometer.
	\return True if the Barometer flag is set
*/
int XsDeviceCapabilities_hasBarometer(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_Baro) > 0;
}

/*!	\brief	Test if the device has an operational GNSS receiver.
	\return True if the GNSS flag is set
*/
int XsDeviceCapabilities_hasGnss(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_Gnss) > 0;
}

/*!	\brief	Test if the device is an IMU.
	\return True if the IMU flag is set
*/
int XsDeviceCapabilities_isImu(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_Imu) > 0;
}

/*!	\brief	Test if the device has a VRU.
	\return True if the VRU flag is set
*/
int XsDeviceCapabilities_isVru(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_Vru) > 0;
}

/*!	\brief	Test if the device has an AHRS.
	\return True if the AHRS flag is set
*/
int XsDeviceCapabilities_isAhrs(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_Ahrs) > 0;
}

/*!	\brief	Test if the device is a GNSS/INS.
	\return True if the GNSS/INS flag is set
*/
int XsDeviceCapabilities_isGnssIns(const struct XsDeviceCapabilities* thisPtr)
{
	return (thisPtr->m_flags & XDC_GnssIns) > 0;
}

/*! @} */
