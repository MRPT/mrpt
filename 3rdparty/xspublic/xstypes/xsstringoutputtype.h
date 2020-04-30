
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

#ifndef XSSTRINGOUTPUTTYPE_H
#define XSSTRINGOUTPUTTYPE_H

/*!	\addtogroup enums Global enumerations
	@{
*/
//! String output types
enum XsStringOutputType {
	 XSOT_None		= 0x0000
	,XSOT_HCHDM		= 0x0001 //!< NMEA string with Magnetic Heading
	,XSOT_HCHDG		= 0x0002 //!< NMEA string with Heading and Magnetic Variation
	,XSOT_TSS2		= 0x0004 //!< Proprietry string with Heading, Heave, Roll and Pitch
	,XSOT_PHTRO		= 0x0008 //!< Proprietry NMEA string with Pitch and Roll
	,XSOT_PRDID		= 0x0010 //!< Proprietry NMEA string with Pitch, Roll and Heading
	,XSOT_EM1000	= 0x0020 //!< Binary format suitable for use with Simrad EM1000 mulitibeam sounders with Roll, Pitch, Heave and Heading
	,XSOT_PSONCMS	= 0x0040 //!< NMEA string with Xsens Compass Motion Sensor information
	,XSOT_HCMTW		= 0x0080 //!< NMEA string with (water) Temperature
	,XSOT_HEHDT		= 0x0100 //!< NMEA string with True Heading
	,XSOT_HEROT		= 0x0200 //!< NMEA string with Rate of Turn
	,XSOT_GPGGA		= 0x0400 //!< NMEA string with Global Positioning system fix data
	,XSOT_PTCF		= 0x0800 //!< NMEA string with motion data
	,XSOT_XSVEL		= 0x1000 //!< Proprietry NMEA string with velocity data
	,XSOT_GPZDA		= 0x2000 //!< NMEA string with date and time
	,XSOT_GPRMC		= 0x4000 //!< NMEA string with recommended minimum specific GPS/Transit data
};
/*! @} */
typedef enum XsStringOutputType XsStringOutputType;

#endif
