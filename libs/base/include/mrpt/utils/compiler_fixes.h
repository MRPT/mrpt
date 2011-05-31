/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
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
	#pragma warning(disable:4267)
	#pragma warning(disable:4290) // Visual C++ does not implement decl. specifiers: throw(A,B,...)
	#pragma warning(disable:4251) // Visual C++ 2003+ warnings on STL classes when exporting to DLL...
	#pragma warning(disable:4275)
	#if (_MSC_VER >= 1400 )
		// MS believes they have the right to deprecate functions in the C++ Standard STL... disable their warnings:
		#ifndef _SCL_SECURE_NO_WARNINGS
			#define _SCL_SECURE_NO_WARNINGS
		#endif
		#pragma warning(disable:4996)
		// For the new secure library in VC++8
		#if !defined(_CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES)
			#define _CRT_SECURE_CPP_OVERLOAD_STANDARD_NAMES 1
		#endif
	#endif
#endif

// Avoid conflicting declaration of max macro in windows headers
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

// Enable leak memory debugging:
#if defined(_DEBUG) && defined(_MSC_VER) && (_MSC_VER>=1400)
	#define _CRTDBG_MAP_ALLOC
	#include <stdlib.h>
	#include <crtdbg.h>
#endif

