
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

#ifndef XSENS_MATH_THROW_H
#define XSENS_MATH_THROW_H

#include <xstypes/xstypesconfig.h>

#ifndef XSENS_NO_EXCEPTIONS
	#include <xstypes/xsexception.h>
#endif

#ifdef XSENS_NO_EXCEPTIONS
	// support for exceptions is disabled, just do whatever assert(0) does
	#ifdef XSENS_DEBUG
		#include <assert.h>
		#define XM_THROW(a)			XSENS_FW_ASSERT_FUNC(a, __FILE__, (unsigned) __LINE__)
		#define XM_THROW_DEFINED	1
	#else
		#define XM_THROW(a)			((void) 0)
		#define XM_THROW_DEFINED	0
	#endif
#else
	#define XM_THROW_DEFINED	1
	#ifdef XSEXCEPTION_H
		#ifdef _MSC_VER
			#define XETHROW(a)	throw XsException(XRV_ERROR, XsString(__FUNCTION__ " ") << XsString(a))
		#elif defined __GNUC__
			#define XETHROW(a)	throw XsException(XRV_ERROR, XsString(__PRETTY_FUNCTION__) << " " << XsString(a))
		#else
			#define XETHROW(a)	throw XsException(XRV_ERROR, XsString(__func__) << " " << XsString(a))
		#endif
	#else
		#define XETHROW(a)	throw (a)
	#endif

	#if defined(XSENS_DEBUG) && defined(_WIN32) // && !defined(_WIN64) unclear why this clause was added in xsens_math.h rev 871 by RZA
		#define XM_THROW(a) do { xsens::DebugTools::mathThrowBreakFunc(); XETHROW(a); } while(0)
	#else
		#define XM_THROW(a) do { XETHROW(a); } while(0)
	#endif
#endif

namespace xsens
{
	namespace DebugTools
	{
		void mathThrowBreakFunc();
		void enableFloatingPointExceptions();
		void disableFloatingPointExceptions();
	} // namespace DebugTools
} // namespace xsens

#endif
