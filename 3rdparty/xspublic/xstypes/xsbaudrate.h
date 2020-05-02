
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

#ifndef XSBAUDRATE_H
#define XSBAUDRATE_H

#ifdef _WIN32
#	include <windows.h>
#else
#	include <termios.h>
#	ifndef	B460800
#		undef	B230400
#		define	B230400	0010003
#		define  B460800 0010004
#		define  B921600 0010007
#	endif
#endif

/*!	\addtogroup enums Global enumerations
	@{
*/
#ifdef _WIN32

//AUTO namespace xstypes {
/*! \brief Communication speed. */
enum XsBaudRate {
	XBR_Invalid					= 0,			//!< Not a valid baud rate
	XBR_4800					= CBR_4800,		//!< 4k8 (4800 bps)
	XBR_9600					= CBR_9600,		//!< 9k6 (9600 bps)
//	XBR_14k4					= CBR_14400,
	XBR_19k2					= CBR_19200,	//!< 19k2 (19200 bps)
//	XBR_28k8					= 28800,
	XBR_38k4					= CBR_38400,	//!< 38k4 (38400 bps)
	XBR_57k6					= CBR_57600,	//!< 57k6 (57600 bps)
	XBR_115k2					= CBR_115200,	//!< 115k2 (115200 bps)
	XBR_230k4					= 230400,		//!< 230k4 (230400 bps)
	XBR_460k8					= 460800,		//!< 460k8 (460800 bps)
	XBR_921k6					= 921600,		//!< 921k6 (921600 bps)
	XBR_2000k					= 2000000,		//!< 2000k0 (2000000 bps)
	XBR_3500k					= 3500000,		//!< 3500k0 (3500000 bps)
	XBR_4000k					= 4000000		//!< 4000k0 (4000000 bps)
};
//AUTO }

#else

/*! \brief Communication speed. */
enum XSNOCOMEXPORT XsBaudRate {

	// support high baudrates on MAC OS X
#ifndef SWIG

#ifndef	B2000000
    #define B2000000	2000000
    #endif
    #ifndef	B3500000
    #define B3500000	3500000
    #endif
    #ifndef	B4000000
    #define B4000000	4000000
    #endif
#endif
	XBR_Invalid					= 0,			//!< Not a valid baud rate

	XBR_4800					= B4800,		//!< 4k8 (4800 bps)
	XBR_9600					= B9600,		//!< 9k6 (9600 bps)
//	XBR_14k4					= B14400,
	XBR_19k2					= B19200,		//!< 19k2 (19200 bps)
//	XBR_28k8					= B28800,
	XBR_38k4					= B38400,		//!< 38k4 (38400 bps)
	XBR_57k6					= B57600,		//!< 57k6 (57600 bps)
	XBR_115k2					= B115200,		//!< 115k2 (115200 bps)
	XBR_230k4					= B230400,		//!< 230k4 (230400 bps)
	XBR_460k8					= B460800,		//!< 460k8 (460800 bps)
	XBR_921k6					= B921600,		//!< 921k6 (921600 bps)
#ifndef SWIG
	XBR_2000k					= B2000000,		//!< 2000k0 (2000000 bps)
	XBR_3500k					= B3500000,		//!< 3500k0 (3500000 bps)
	XBR_4000k					= B4000000		//!< 4000k0 (4000000 bps)
#else
	XBR_2000k					= 2000000,		//!< 2000k0 (2000000 bps)
	XBR_3500k					= 3500000,		//!< 3500k0 (3500000 bps)
	XBR_4000k					= 4000000		//!< 4000k0 (4000000 bps)
#endif
};
/*! \brief Communication speed. */
typedef enum XsBaudRate XsBaudRate;
#endif
/*! @} */

#endif
