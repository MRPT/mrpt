/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/system/backtrace.h>

#ifdef MRPT_OS_WINDOWS
	#define WIN32_LEAN_AND_MEAN
	#include <windows.h>
	#include <DbgHelp.h>
#else
	#include <execinfo.h>
	#include <dlfcn.h>    // dladdr()
	#include <cxxabi.h>   // __cxa_demangle()
	#include <cstdlib>
	#include <string>
	#include <sstream>
	#include <iostream>
#endif

void mrpt::system::getCallStackBackTrace(TCallStackBackTrace &out_bt)
{
	out_bt.backtrace_levels.clear();
	const unsigned int framesToSkip = 1; // skip *this* function from the backtrace
	const unsigned int framesToCapture = 64;

#ifdef MRPT_OS_WINDOWS
	void* backTrace[framesToCapture]{};

	SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
	const HANDLE hProcess = GetCurrentProcess();
	if (!SymInitialize(hProcess, nullptr /* UserSearchPath  */, TRUE /*fInvadeProcess*/))
	{
		std::cerr << "[mrpt::system::getCallStackBackTrace] Error in SymInitialize()!" << std::endl;
		return;
	}

	char buffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME * sizeof(TCHAR)];
	PSYMBOL_INFO pSymbol = (PSYMBOL_INFO)buffer;
	pSymbol->SizeOfStruct = sizeof(SYMBOL_INFO);
	pSymbol->MaxNameLen = MAX_SYM_NAME;

	const USHORT nFrames = CaptureStackBackTrace(framesToSkip, framesToCapture, backTrace, nullptr);
	for (unsigned int i = 0; i < nFrames; i++)
	{
		TCallStackEntry cse;
		cse.address = backTrace[i];

		if (!SymFromAddr(hProcess, (DWORD64)cse.address, nullptr, pSymbol))
		{
			cse.symbolName = "???";
			cse.symbolNameOriginal = "???";
			out_bt.backtrace_levels.emplace_back(cse);
			continue;
		}
		SYMBOL_INFO &si = *pSymbol;

		cse.symbolNameOriginal = si.Name;
		char undecorated_name[1024];
		if (!UnDecorateSymbolName(si.Name, undecorated_name, sizeof(undecorated_name), UNDNAME_COMPLETE))
		{
			cse.symbolName = cse.symbolNameOriginal;
		}
		else
		{
			cse.symbolName = std::string(undecorated_name);
		}

		out_bt.backtrace_levels.emplace_back(cse);
	}
#else
	// Based on: https://gist.github.com/fmela/591333
	void *callstack[framesToCapture];
	const int nMaxFrames = sizeof(callstack) / sizeof(callstack[0]);
	char buf[1024];
	int nFrames = ::backtrace(callstack, nMaxFrames);
	char **symbols = ::backtrace_symbols(callstack, nFrames);

	std::ostringstream trace_buf;
	for (unsigned int i = framesToSkip; i < nFrames; i++)
	{
		Dl_info info;
		if (dladdr(callstack[i], &info) && info.dli_sname) 
		{
			char *demangled = NULL;
			int status = -1;
			if (info.dli_sname[0] == '_') 
			{
				demangled = abi::__cxa_demangle(info.dli_sname, NULL, 0, &status);
			}
			snprintf(buf, sizeof(buf), "%-3d %*p %s + %zd\n",
				i, int(2 + sizeof(void*) * 2), callstack[i],
				status == 0 ? demangled :
				info.dli_sname == 0 ? symbols[i] : info.dli_sname,
				(char *)callstack[i] - (char *)info.dli_saddr);
			free(demangled);
		}
		else {
			snprintf(buf, sizeof(buf), "%-3d %*p %s\n",
				i, int(2 + sizeof(void*) * 2), callstack[i], symbols[i]);
		}
		trace_buf << buf;
	}
	free(symbols);
	MRPT_TODO("finish this impl:");
	std::cout << "stack:\n" << trace_buf.str();
#endif
}

mrpt::system::TCallStackBackTrace::TCallStackBackTrace()
{
}

std::string mrpt::system::TCallStackBackTrace::asString() const
{
	std::ostringstream trace_buf;
	trace_buf << "Backtrace:" << std::endl;
	for (unsigned int i = 0; i < this->backtrace_levels.size(); i++)
	{
		trace_buf << mrpt::format("[%2d] %*p %s", i, int(2 + sizeof(void*) * 2), backtrace_levels[i].address, backtrace_levels[i].symbolName.c_str()) << std::endl;
	}
	return trace_buf.str();
}
