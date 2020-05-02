
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

#ifndef XSBAUDCODE_H
#define XSBAUDCODE_H

#include "xstypesconfig.h"

/*! \brief Internal baud rate configuration codes
*/

enum XSNOCOMEXPORT XsBaudCode {
	// Baudrate codes for SetBaudrate message
	XBC_4k8           = 0x0B,		//!< 4k8 (4800 bps)
	XBC_9k6           = 0x09,		//!< 9k6 (9600 bps)
//	XBC_14k4          = 0x08,
	XBC_19k2          = 0x07,		//!< 19k2 (19200 bps)
//	XBC_28k8          = 0x06,
	XBC_38k4          = 0x05,		//!< 38k4 (38400 bps)
	XBC_57k6          = 0x04,		//!< 57k6 (57600 bps)
//	XBC_76k8          = 0x03,
	XBC_115k2         = 0x02,		//!< 115k2 (115200 bps)
	XBC_230k4         = 0x01,		//!< 230k4 (230400 bps)
	XBC_460k8         = 0x00,		//!< 460k8 (460800 bps)
	XBC_921k6         = 0x0A,		//!< 921k6 (921600 bps). Only usable from MTi/x FW 2.4.6
	XBC_921k6Legacy   = 0x80,		//!< 921k6 (921600 bps)
	XBC_2MegaBaud     = 0x0C,		//!< 2000k0 (2000000 bps)
	XBC_3_5MegaBaud   = 0x0E,		//!< 3500k0 (3500000 bps)
	XBC_4MegaBaud	  = 0x0D,		//!< 4000k0 (4000000 bps)
	XBC_Invalid       = 0xFF		//!< Not a valid baud rate
};

#endif
