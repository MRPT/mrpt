
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

#ifndef XSBAUD_H
#define XSBAUD_H

#include "xstypesconfig.h"


/*!	\addtogroup enums Global enumerations
	@{
*/

#include "xsbaudcode.h"
#include "xsbaudrate.h"

/*! @} */

typedef enum XsBaudCode XsBaudCode;
typedef enum XsBaudRate XsBaudRate;

#ifdef __cplusplus
extern "C" {
#endif

XSTYPES_DLL_API XsBaudRate XsBaud_codeToRate(XsBaudCode baudcode);
XSTYPES_DLL_API XsBaudCode XsBaud_rateToCode(XsBaudRate baudrate);
XSTYPES_DLL_API int XsBaud_rateToNumeric(XsBaudRate baudrate);
XSTYPES_DLL_API XsBaudRate XsBaud_numericToRate(int numeric);

#ifdef __cplusplus
} // extern "C"

/*! \namespace XsBaud
	\brief Namespace for Baud rate and Baud code constants and conversions
*/
namespace XsBaud {
	/*! \copydoc XsBaud_codeToRate */
	inline XsBaudRate codeToRate(XsBaudCode baudcode)
	{
		return XsBaud_codeToRate(baudcode);
	}
	/*! \copydoc XsBaud_rateToCode */
	inline XsBaudCode rateToCode(XsBaudRate baudrate)
	{
		return XsBaud_rateToCode(baudrate);
	}
	/*! \copydoc XsBaud_rateToNumeric */
	inline int rateToNumeric(XsBaudRate baudrate)
	{
		return XsBaud_rateToNumeric(baudrate);
	}
	/*! \copydoc XsBaud_numericToRate*/
	inline XsBaudRate numericToRate(int numeric)
	{
		return XsBaud_numericToRate(numeric);
	}
}

#ifndef XSENS_NO_STL
#include <ostream>

namespace std
{
	template<typename _CharT, typename _Traits>
	basic_ostream<_CharT, _Traits>& operator<<(basic_ostream<_CharT, _Traits>& o, XsBaudRate const& xd)
	{
		return (o << XsBaud::rateToNumeric(xd));
	}
}
#endif

#endif

#endif
