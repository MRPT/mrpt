
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

#ifndef XSSYNCLINEMK4_H
#define XSSYNCLINEMK4_H

#include <xstypes/xssyncline.h>

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Synchronization line identifiers for the Mk4 devices, only to be used directly in Xbus messages */
enum SyncLineMk4
{
	XSL4_ClockIn = 0,		//!< External clock sync \ref XSL_ClockIn
	XSL4_GnssClockIn = 1,	//!< GNSS clock sync \ref XSL_GnssClockIn
	XSL4_In = 2,			//!< Send data line \ref XSL_In1
	XSL4_BiIn = 3,			//!< Bidirectional sync line, configured as input \ref XSL_Bi1In
	XSL4_BiOut = 4,			//!< Bidirectional sync line, configured as output \ref XSL_Bi1Out
	XSL4_ExtTimepulseIn = 5,//!< External Timepulse input \ref XSL_ExtTimepulseIn
	XSL4_ReqData = 6,		//!< Serial data sync option, use XMID_ReqData message id for this \ref XSL_ReqData
	XSL4_Gnss1Pps = 8,		//!< GNSS 1PPS sync line \ref XSL_Gnss1Pps

	XSL4_Invalid
};
/*! @} */
typedef enum SyncLineMk4 SyncLineMk4;

#ifdef __cplusplus
extern "C" {
#endif

XsSyncLine xsl4ToXsl(SyncLineMk4 mk4Line);
SyncLineMk4 xslToXsl4(XsSyncLine line);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
