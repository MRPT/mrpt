
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

#ifndef XSDEVICEOPTIONFLAG_H
#define XSDEVICEOPTIONFLAG_H

/*!	\addtogroup enums Global enumerations
@{
*/
/*! \brief Used to enable or disable some device options
	\sa XsDevice::setDeviceOptionFlags
	\note Not all devices support all options.
*/
enum XsDeviceOptionFlag
{
	XDOF_DisableAutoStore				= 0x00000001,	//!< When set to 1, automatic writing of configuration will be disabled.
	XDOF_DisableAutoMeasurement			= 0x00000002,	//!< When set to 1, the MT will stay in Config Mode upon start up.
	XDOF_EnableBeidou					= 0x00000004,	//!< When set to 1, enables Beidou, disables GLONASS (MTi-G).
	XDOF_DisableGps						= 0x00000008,	//!< When set to 1, disables GPS (MTi-G).
	XDOF_EnableAhs						= 0x00000010,	//!< When set to 1, the MTi will have Active Heading Stabilization (AHS) enabled.
	XDOF_EnableOrientationSmoother		= 0x00000020,	//!< When set to 1, the MTi will have Orientation Smoother enabled. Only applicable to MTi-G-710 and MTi-7
	XDOF_EnableConfigurableBusId		= 0x00000040,	//!< When set to 1, allows to configure the BUS ID.
	XDOF_EnableInrunCompassCalibration	= 0x00000080,	//!< When set to 1, the MTi will have In-run Compass Calibration (ICC) enabled.
	XDOF_DisableSleepMode				= 0x00000100,	//!< When set to 1, an MTw will not enter sleep mode after a scan timeout. It will scan indefinitely.
	XDOF_EnableConfigMessageAtStartup	= 0x00000200,	//!< When set to 1, the MT will send the Configuration to the Master at start-up
	XDOF_EnableColdFilterResets			= 0x00000400,	//!< When set to 1, The MT performs a cold filter reset every time it goes to measurement

	XDOF_None							= 0x00000000,	//!< When set to 1, disables all option flags.
	XDOF_All							= 0x7FFFFFFF	//!< When set to 1, enables all option flags.
};
/* @} */
typedef enum  XsDeviceOptionFlag XsDeviceOptionFlag;

#ifdef __cplusplus
//! \brief Logical OR operator for XsDeviceOptionFlag values
inline XsDeviceOptionFlag operator | (XsDeviceOptionFlag a, XsDeviceOptionFlag b)
{
	return (XsDeviceOptionFlag) ((int)a | (int)b);
}

//! \brief Logical AND operator for XsDeviceOptionFlag values
inline XsDeviceOptionFlag operator & (XsDeviceOptionFlag a, XsDeviceOptionFlag b)
{
	return (XsDeviceOptionFlag) ((int)a & (int)b);
}

//! \brief Logical XOR operator for XsDeviceOptionFlag values
inline XsDeviceOptionFlag operator ^ (XsDeviceOptionFlag a, XsDeviceOptionFlag b)
{
	return (XsDeviceOptionFlag) ((int)a ^ (int)b);
}
//! \brief Logical NEG operator for XsDeviceOptionFlag values
inline XsDeviceOptionFlag operator ~ (XsDeviceOptionFlag a)
{
	return (XsDeviceOptionFlag) (~(int)a);
}
#endif

#endif
