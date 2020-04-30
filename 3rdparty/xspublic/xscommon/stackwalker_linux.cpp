
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

#include "stackwalker_linux.h"

#include <stdlib.h>

#ifdef ANDROID
#include <android/log.h>
#elif defined(USING_UNWIND_LIB)
#define UNW_LOCAL_ONLY
#include <libunwind.h>
#include <cxxabi.h>
#include <cstdio>
#endif

#ifdef __arm__
#define UNW_PRINTF_POINTER "0x%x: "
#define UNW_PRINTF_NAME_AND_POINTER " (%s+0x%x)"
#else
	#include <stdint.h>
	#if UINTPTR_MAX == 0xffffffff
		/* 32-bit */
		#define UNW_PRINTF_POINTER "0x%x: "
		#define UNW_PRINTF_NAME_AND_POINTER " (%s+0x%x)"
	#elif UINTPTR_MAX == 0xffffffffffffffff
		/* 64-bit */
		#define UNW_PRINTF_POINTER "0x%lx: "
		#define UNW_PRINTF_NAME_AND_POINTER " (%s+0x%lx)"
	#else
		/* wtf */
	#endif
#endif

StackWalker::StackWalker()
{
}

StackWalker::~StackWalker()
{
}

void StackWalker::ShowCallstack()
{
#ifdef ANDROID
	__android_log_print(ANDROID_LOG_WARN, "stackwalker", "stack trace is not available");
#elif defined(USING_UNWIND_LIB)
	unw_context_t context;
	unw_getcontext(&context);

	unw_cursor_t cursor;
	unw_init_local(&cursor, &context);

	while (unw_step(&cursor) > 0)
	{
		unw_word_t instructionPointer;
		unw_get_reg(&cursor, UNW_REG_IP, &instructionPointer);

		if (instructionPointer == 0)
			break;

		static const int logLineSize = 256;
		char logLine[logLineSize];
		int offset = std::snprintf(logLine, logLineSize, UNW_PRINTF_POINTER, instructionPointer);

		char symbol[logLineSize - 20];
		unw_word_t symbolOffset;
		if (unw_get_proc_name(&cursor, symbol, sizeof(symbol), &symbolOffset) == 0)
		{
			int status;
			char* demangled = abi::__cxa_demangle(symbol, nullptr, nullptr, &status);

			char* symbolName = symbol;
			if (status == 0)
				symbolName = demangled;

			std::snprintf(&logLine[offset], logLineSize - offset, UNW_PRINTF_NAME_AND_POINTER, symbolName, symbolOffset);
			std::free(demangled);
		}
		else
		{
			std::snprintf(&logLine[offset], logLineSize - offset, " (unable to retrieve symbol name)");
		}

		OnOutput(logLine);
	}
#else
	OnOutput("Stack trace is not available");
#endif
}
