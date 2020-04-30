
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

#ifndef XSOPTION_H
#define XSOPTION_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Xda options, used to control the kind of data processing done by XDA
	\details These options are used to specify whether XDA should compute certain kinds of data from
	available other data. XsOptions can be logically ORed together
*/
//AUTO namespace xstypes {
enum XsOption {
	  XSO_None = 0					//!< No option
	, XSO_Calibrate = 0x0001		//!< Compute calibrated inertial data from raw data and temperature
	, XSO_Orientation = 0x0002		//!< Compute orientation, the orientation is typically only computed in one stream. If not specified the system will decide: when reading from file it will use XSO_OrientationInBufferedStream, otherwise XSO_OrientationInLiveStream.

	, XSO_KeepLastLiveData = 0x0004				//!< Keep the last available live data in a cache so XsDevice::lastAvailableLiveData will work
	, XSO_RetainLiveData = 0x0008				//!< Keep the live data in a cache so it can be accessed through XsDevice::getDataPacketByIndex or XsDevice::takeFirstDataPacketInQueue. This option is mutually exclusive with XSO_RetainBufferedData. If both are set, XSO_RetainBufferedData will be used.
	, XSO_RetainBufferedData = 0x0010			//!< Keep the buffered data in a cache so it can be accessed through XsDevice::getDataPacketByIndex or XsDevice::takeFirstDataPacketInQueue. This option is mutually exclusive with XSO_RetainLiveData. If both are set, XSO_RetainBufferedData will be used.
	, XSO_OrientationInLiveStream = 0x0020		//!< Compute orientation in the live stream. This is no longer (since version 4.9.2) mutually exclusive with XSO_OrientationInBufferedStream, but unless they're both explicitly set, the system will choose only one stream to use. Please note that this option does not do anything unless XSO_Orientation is also enabled.
	, XSO_OrientationInBufferedStream = 0x0040	//!< Compute orientation in the buffered stream. This is no longer (since version 4.9.2) mutually exclusive with XSO_OrientationInLiveStream, but unless they're both explicitly set, the system will choose only one stream to use. Please note that this option does not do anything unless XSO_Orientation is also enabled.

	, XSO_ApplyOrientationResetToCalData = 0x0080	//!< Apply orientation reset to calibrated acc, gyr and mag (object reset only) and heading reset to free acc. Default is enabled for the MTw family, disabled for others

	, XSO_InterpolateMissingData = 0x1000			//!< When set, any gaps in the data streams of child devices will be filled with interpolated data. This is only for applications that require data for each sample counter. Not recommended when XSO_SkipDataBundling is disabled.
	, XSO_SkipDataBundling = 0x2000					//!< When set, the onAll...DataAvailable callbacks will not be called by the master device. This prevents some internal buffering.
	, XSO_ExpectNoRetransmissionsInFile = 0x4000	//!< When set and reading a file, missing data is immediately treated as unavailable. The default behaviour is to read further in the file to see if the data was retransmitted.
	, XSO_Reserved = 0x8000							//!< Reserved for internal use

	, XSO_All = 0xFFFF				//!< All options, note that setting 'all options' is not valid, but it is useful for clearing all options
};

/*! @} */
typedef enum XsOption XsOption;

//AUTO }

#ifdef __cplusplus
//! \brief Logical OR operator for XsOption values
inline XsOption operator | (XsOption a, XsOption b)
{
	return (XsOption) ((int)a | (int)b);
}
//! \brief Logical AND operator for XsOption values
inline XsOption operator & (XsOption a, XsOption b)
{
	return (XsOption) ((int)a & (int)b);
}
//! \brief Logical XOR operator for XsOption values
inline XsOption operator ^ (XsOption a, XsOption b)
{
	return (XsOption) ((int)a ^ (int)b);
}
//! \brief Logical NEG operator for XsOption values
inline XsOption operator ~ (XsOption a)
{
	return (XsOption) (~(int)a);
}
//! \brief Return the option with mutually exclusive values 'fixed'
inline XsOption XsOption_purify(XsOption a)
{
	if ((a & XSO_RetainLiveData) && (a & XSO_RetainBufferedData))
		a = a & ~XSO_RetainLiveData;

	return a;
}
#endif

#endif
