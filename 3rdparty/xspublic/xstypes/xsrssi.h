
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

#ifndef XSRSSI_H
#define XSRSSI_H

#include "xstypesconfig.h"

#define XS_RSSI_MAX		(128)
#define XS_RSSI_UNKNOWN	(-XS_RSSI_MAX)

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API int XsRssi_max(void);
XSTYPES_DLL_API int XsRssi_unknown(void);
XSTYPES_DLL_API int XsRssi_unbiased(int raw);

#ifdef __cplusplus
} // extern "C"
namespace XsRssi {
	/*! \brief The bias on biased RSSI values. */
	static const int bias = XS_RSSI_MAX;

	/*! \brief The maximum RSSI value. */
	static const int max = XS_RSSI_MAX;
	/*! \brief The RSSI value that was reserved for when the RSSI is unknown. */
	static const int unknown = XS_RSSI_UNKNOWN;

	/*! \brief The maximum unbiased RSSI value. */
	static const int maxUnbiased = XS_RSSI_MAX + XS_RSSI_MAX;
	/*! \brief The RSSI value that was reserved for when the RSSI is unknown (unbiased). */
	static const int unknownUnbiased = XS_RSSI_UNKNOWN + XS_RSSI_MAX;

	/*! \copydoc XsRssi_unbiased */
	inline int unbiased(int raw)
	{
		return XsRssi_unbiased(raw);
	}
}
#endif

#endif
