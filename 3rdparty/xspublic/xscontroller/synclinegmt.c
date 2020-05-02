
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

#include "synclinegmt.h"

/*! \addtogroup cinterface C Interface
	@{
*/

//! \brief Translate an SyncLineGmt into a generic XsSyncLine
XsSyncLine xslgmtToXsl(SyncLineGmt mk4Line)
{
	switch (mk4Line)
	{
	case XSLGMT_ClockIn:		return XSL_ClockIn;
	case XSLGMT_GnssClockIn:	return XSL_GnssClockIn;
	case XSLGMT_ReqData:		return XSL_ReqData;
	case XSLGMT_In1:			return XSL_In1;
	case XSLGMT_Out:			return XSL_Out1;
	case XSLGMT_BiIn:			return XSL_Bi1In;
	case XSLGMT_BiOut:			return XSL_Bi1Out;
	case XSLGMT_Gnss1Pps:		return XSL_Gnss1Pps;
	case XSLGMT_In2:			return XSL_In2;
	default:					return XSL_Invalid;
	}
}

//! \brief Translate an XsSyncLine into a SyncLineGmt
SyncLineGmt xslToXslgmt(XsSyncLine line)
{
	switch (line)
	{
	case XSL_ClockIn:			return XSLGMT_ClockIn;
	case XSL_GnssClockIn:		return XSLGMT_GnssClockIn;
	case XSL_In1:				return XSLGMT_In1;
	case XSL_Out1:				return XSLGMT_Out;
	case XSL_ReqData:			return XSLGMT_ReqData;
	case XSL_Bi1In:				return XSLGMT_BiIn;
	case XSL_Bi1Out:			return XSLGMT_BiOut;
	case XSL_Gnss1Pps:			return XSLGMT_Gnss1Pps;
	case XSL_In2:				return XSLGMT_In2;
	default:					return XSLGMT_Invalid;
	}
}

/*! @} */
