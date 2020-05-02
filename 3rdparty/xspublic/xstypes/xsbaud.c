
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

#include "xsbaud.h"

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \brief Convert an Xsens baud code to XsBaudRate

	\param baudcode : The code to convert to an XsBaudRate

	\returns An XsBaudrate
*/
XsBaudRate XsBaud_codeToRate(XsBaudCode baudcode)
{
	switch(baudcode)
	{
	case XBC_4k8:		  return XBR_4800;
	case XBC_9k6:		  return XBR_9600;
//	case XBC_14k4:		  return XBR_14k4;
	case XBC_19k2:		  return XBR_19k2;
//	case XBC_28k8:		  return XBR_28k8;
	case XBC_38k4:		  return XBR_38k4;
	case XBC_57k6:		  return XBR_57k6;
//	case XBC_76k8:		  return XBR_76k8;
	case XBC_115k2:		  return XBR_115k2;
	case XBC_230k4:		  return XBR_230k4;
	case XBC_460k8:		  return XBR_460k8;
	case XBC_921k6:
	case XBC_921k6Legacy: return XBR_921k6;
	case XBC_2MegaBaud:	  return XBR_2000k;
	case XBC_3_5MegaBaud: return XBR_3500k;
	case XBC_4MegaBaud:	  return XBR_4000k;
	default:              return XBR_Invalid;
	}
}

/*! \brief Convert a XsBaudRate to an Xsens baud code

	\param baudrate : The code to convert to an XsBaudCode

	\returns An XsBaudCode
*/
XsBaudCode XsBaud_rateToCode(XsBaudRate baudrate)
{
	switch(baudrate)
	{
	case XBR_4800:	return XBC_4k8;
	case XBR_9600:	return XBC_9k6;
//	case XBR_14k4:	return XBC_14k4;
	case XBR_19k2:	return XBC_19k2;
//	case XBR_28k8:	return XBC_28k8;
	case XBR_38k4:	return XBC_38k4;
	case XBR_57k6:	return XBC_57k6;
//	case XBR_76k8:	return XBC_76k8;
	case XBR_115k2:	return XBC_115k2;
	case XBR_230k4:	return XBC_230k4;
	case XBR_460k8:	return XBC_460k8;
	case XBR_921k6:	return XBC_921k6Legacy;
	case XBR_2000k:	return XBC_2MegaBaud;
	case XBR_3500k: return XBC_3_5MegaBaud;
	case XBR_4000k: return XBC_4MegaBaud;
	default:		return XBC_Invalid;
	}
}

/*! \brief Convert a XsBaudrate to a numeric baudrate in bps

	\param baudrate : The XsBaudRate to convert to a numeric baudrate

	\returns A baudrate in bps
*/
int XsBaud_rateToNumeric(XsBaudRate baudrate)
{
	switch(baudrate)
	{
	case XBR_4800:	return 4800;
	case XBR_9600:	return 9600;
//	case XBR_14k4:	return 14400;
	case XBR_19k2:	return 19200;
//	case XBR_28k8:	return 28800;
	case XBR_38k4:	return 38400;
	case XBR_57k6:	return 57600;
//	case XBR_76k8:	return 76800;
	case XBR_115k2:	return 115200;
	case XBR_230k4:	return 230400;
	case XBR_460k8:	return 460800;
	case XBR_921k6:	return 921600;
	case XBR_2000k:	return 2000000;
	case XBR_3500k: return 3500000;
	case XBR_4000k: return 4000000;
	default:		return 0;
	}
}

/*! \brief Convert a numeric baudrate in bps to XsBaudrate

	\param numeric : The numeric baudrate to convert to XsBaudRate

	\returns A XsBaudrate
*/
XsBaudRate XsBaud_numericToRate(int numeric)
{
	switch(numeric)
	{
	case 4800:		return XBR_4800;
	case 9600:		return XBR_9600;
//	case 14400:		return XBR_14k4;
	case 19200:		return XBR_19k2;
//	case 28800:		return XBR_28k8;
	case 38400:		return XBR_38k4;
	case 57600:		return XBR_57k6;
//	case 76800:		return XBR_76k8;
	case 115200:	return XBR_115k2;
	case 230400:	return XBR_230k4;
	case 460800:	return XBR_460k8;
	case 921600:	return XBR_921k6;
	case 2000000:	return XBR_2000k;
	case 3500000:	return XBR_3500k;
	case 4000000:	return XBR_4000k;
	default:		return XBR_Invalid;
	}
}

 /*! @} */
