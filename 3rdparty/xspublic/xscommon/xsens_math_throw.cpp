
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

#include "xsens_math_throw.h"

#ifndef __STDC_VERSION__
#define __STDC_VERSION__	0
#endif

#if __STDC_VERSION__ >= 199901L
#include <fenv.h>
#endif

namespace xsens
{
namespace DebugTools
{

// please note that this will never be called in full release builds!
void mathThrowBreakFunc()
{
	// volatile to prevent the function being optimized away
	volatile int putMathBreakPointHere = 0;
	(void)putMathBreakPointHere;
}

/*! \brief Enable floating point exceptions

	On windows, this requires /fp:except for proper use.
	It also needs implementation and a bit of testing.
*/
void enableFloatingPointExceptions()
{
#ifdef _MSC_VER
//	_clearfp(); // always call _clearfp before enabling/unmasking a FPU exception
//	_controlfp(_EM_INVALID | _EM_DENORMAL | _EM_ZERODIVIDE | _EM_OVERFLOW |
//			 _EM_UNDERFLOW | _EM_INEXACT, _MCW_EM);
#elif __STDC_VERSION__ >= 199901L
	feenableexcept(FE_ALL_EXCEPT);
#endif
}

/*! \brief Disable floating point exceptions

	\see enableFloatingPointExceptions
*/
void disableFloatingPointExceptions()
{
#ifdef _MSC_VER
//	_clearfp();
//	_controlfp(_CW_DEFAULT, ~0);
#elif __STDC_VERSION__ >= 199901L
	feclearexcept(FE_ALL_EXCEPT);
#endif
}

} // namespace DebugTools
} // namespace xsens
