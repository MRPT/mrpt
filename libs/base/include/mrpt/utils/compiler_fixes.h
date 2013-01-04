/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */

#ifndef UTILSDEFS_H
#error "This file is intended for include from utils_defs.h only!"
#endif

/* ------------------------------------
          Disable some warnings
   ------------------------------------ */
#if defined(_MSC_VER)
	#pragma warning(disable:4786) // (Compiler: Visual C++) Disable warning for too long debug names:
	#pragma warning(disable:4503) // (Compiler: Visual C++ 2010) Disable warning for too long decorated name
	#pragma warning(disable:4702) // (Compiler: Visual C++) Disable warning for unreachable code (I don't know why some of these errors appear in the STANDARD LIBRARY headers with Visual Studio 2003!):
	#pragma warning(disable:4244) // (Compiler: Visual C++) Conversion double->float
	#pragma warning(disable:4305)
	#pragma warning(disable:4308) // Disable warning for Eigen3 libs: negative integral converted to unsigned
	#pragma warning(disable:4267)
	#pragma warning(disable:4290) // Visual C++ does not implement decl. specifiers: throw(A,B,...)
	#pragma warning(disable:4251) // Visual C++ 2003+ warnings on STL classes when exporting to DLL...
	#pragma warning(disable:4275)
	#pragma warning(disable:4308) // Disable for Eigen: negative integral constant converted to unsigned int 
	#if (_MSC_VER >= 1400 )
		// MS believes they have the right to deprecate functions in the C++ Standard STL... disable their warnings:
		#ifndef _SCL_SECURE_NO_WARNINGS
			#define _SCL_SECURE_NO_WARNINGS
		#endif
		//#pragma warning(disable:4996)  // Deprecated functions
		// For the new secure library in VC++8
		#if !defined(_CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES)
			#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
		#endif
	#endif
#endif

// Avoid conflicting declaration of max macro in windows headers
#include <algorithm>
#if defined(MRPT_OS_WINDOWS) && !defined(NOMINMAX)
#	define NOMINMAX
#	ifdef max
#		undef	max
#		undef	min
#	endif
#endif

//  We want to avoid defining "max" & "min" as #define's since it create conflicts
//    with methods, variables, etc... with the same name in some compilers.
// Use std::max & std::min for all compilers by default, but for MSVC6 it does not exist:
#if defined(_MSC_VER) && (_MSC_VER<1300)
#	ifndef max
		namespace std
		{
			template<class T> inline const T max(const T& A,const T& B) { return A>B ? A:B; }
			template<class T> inline const T min(const T& A,const T& B) { return A<B ? A:B; }
		}
#	else
#		define  MAX3_MSVC6_VERSION
#	endif
#endif

// Min & Max:
#ifndef MAX3_MSVC6_VERSION
	template<typename T> inline const T  min3(const T& A, const T& B,const T& C) { return std::min<T>(A, std::min<T>(B,C) ); }
	template<typename T> inline const T  max3(const T& A, const T& B,const T& C) { return std::max<T>(A, std::max<T>(B,C) ); }
#else
#	define max3(A,B,C) max(A,max(B,C))
#	define min3(A,B,C) min(A,min(B,C))
#endif
