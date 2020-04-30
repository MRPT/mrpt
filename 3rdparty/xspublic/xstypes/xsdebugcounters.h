
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

#ifndef XSDEBUGCOUNTERS_H
#define XSDEBUGCOUNTERS_H

#include "xstypesconfig.h"

#ifdef XSENS_USE_DEBUG_COUNTERS
#ifdef __cplusplus
extern "C" {
#endif

extern XSTYPES_DLL_API int XsVector_resetDebugCounts();
extern XSTYPES_DLL_API int XsVector_allocCount();
extern XSTYPES_DLL_API int XsVector_freeCount();
extern int XsVector_incAllocCount();
extern int XsVector_incFreeCount();

extern XSTYPES_DLL_API int XsMatrix_resetDebugCounts();
extern XSTYPES_DLL_API int XsMatrix_allocCount();
extern XSTYPES_DLL_API int XsMatrix_freeCount();
extern int XsMatrix_incAllocCount();
extern int XsMatrix_incFreeCount();

extern XSTYPES_DLL_API int XsArray_resetDebugCounts();
extern XSTYPES_DLL_API int XsArray_allocCount();
extern XSTYPES_DLL_API int XsArray_freeCount();
extern int XsArray_incAllocCount();
extern int XsArray_incFreeCount();

#ifdef __cplusplus
} // extern "C"
#endif

#else

inline static int XsVector_resetDebugCounts()	{ return 0; }
inline static int XsVector_allocCount()			{ return 0; }
inline static int XsVector_freeCount()			{ return 0; }
inline static int XsVector_incAllocCount()		{ return 0; }
inline static int XsVector_incFreeCount()		{ return 0; }

inline static int XsMatrix_resetDebugCounts()	{ return 0; }
inline static int XsMatrix_allocCount()			{ return 0; }
inline static int XsMatrix_freeCount()			{ return 0; }
inline static int XsMatrix_incAllocCount()		{ return 0; }
inline static int XsMatrix_incFreeCount()		{ return 0; }

inline static int XsArray_resetDebugCounts()	{ return 0; }
inline static int XsArray_allocCount()			{ return 0; }
inline static int XsArray_freeCount()			{ return 0; }
inline static int XsArray_incAllocCount()		{ return 0; }
inline static int XsArray_incFreeCount()		{ return 0; }

#endif


#endif
