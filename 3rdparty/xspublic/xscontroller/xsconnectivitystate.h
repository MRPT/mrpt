
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

#ifndef XSCONNECTIVITYSTATE_H
#define XSCONNECTIVITYSTATE_H

#include "xscontrollerconfig.h"

#ifdef __cplusplus
#include <ostream>
#endif

/*!	\addtogroup enums Global enumerations
	@{
*/

//AUTO namespace xscontroller {
/*! \brief XsDevice connectivity state identifiers */
enum XsConnectivityState {
	XCS_Disconnected,		/*!< Device has disconnected, only limited informational functionality is available. */
	XCS_Rejected,			/*!< Device has been rejected and is disconnected, only limited informational functionality is available. */
	XCS_PluggedIn,			/*!< Device is connected through a cable. */
	XCS_Wireless,			/*!< Device is connected wirelessly. */
	XCS_WirelessOutOfRange,	/*!< Device was connected wirelessly and is currently out of range. */
	XCS_File,				/*!< Device is reading from a file. */
	XCS_Unknown,			/*!< Device is in an unknown state. */
};

/*! @} */
typedef enum XsConnectivityState XsConnectivityState;
//AUTO }

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Convert the device state to a human readable string */
const char *XsConnectivityState_toString(XsConnectivityState s);

#ifdef __cplusplus
} // extern "C"

#ifndef XSENS_NO_STL
namespace std
{
	template<typename _CharT, typename _Traits>
	basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsConnectivityState const& xs)
	{
		return (o << XsConnectivityState_toString(xs));
	}
}
#endif
#endif

#endif
