/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/core/format.h>
#include <mrpt/system/backtrace.h>
#include <iostream>
#include <sstream>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <DbgHelp.h>
#else
#include <cxxabi.h>  // __cxa_demangle()
#include <dlfcn.h>  // dladdr()
#include <execinfo.h>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#endif

void mrpt::system::getCallStackBackTrace(TCallStackBackTrace& out_bt)
{
	out_bt.backtrace_levels.clear();
	const unsigned int framesToSkip =
		1;  // skip *this* function from the backtrace
	const unsigned int framesToCapture = 64;

#ifdef _WIN32
	void* backTrace[framesToCapture]{};

	SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
	const HANDLE hProcess = GetCurrentProcess();
	if (!SymInitialize(
			hProcess, nullptr /* UserSearchPath  */, TRUE /*fInvadeProcess*/))
	{
		std::cerr
			<< "[mrpt::system::getCallStackBackTrace] Error in SymInitialize()!"
			<< std::endl;
		return;
	}

	char buffer[sizeof(SYMBOL_INFO) + MAX_SYM_NAME * sizeof(TCHAR)];
	PSYMBOL_INFO pSymbol = (PSYMBOL_INFO)buffer;
	pSymbol->SizeOfStruct = sizeof(SYMBOL_INFO);
	pSymbol->MaxNameLen = MAX_SYM_NAME;

	const USHORT nFrames = CaptureStackBackTrace(
		framesToSkip, framesToCapture, backTrace, nullptr);
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
		SYMBOL_INFO& si = *pSymbol;

		cse.symbolNameOriginal = si.Name;
		char undecorated_name[1024];
		if (!UnDecorateSymbolName(
				si.Name, undecorated_name, sizeof(undecorated_name),
				UNDNAME_COMPLETE))
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
	void* callstack[framesToCapture];
	const int nMaxFrames = sizeof(callstack) / sizeof(callstack[0]);
	int nFrames = ::backtrace(callstack, nMaxFrames);
	char** symbols = ::backtrace_symbols(callstack, nFrames);

	for (int i = (int)framesToSkip; i < nFrames; i++)
	{
		TCallStackEntry cse;
		cse.address = callstack[i];

		Dl_info info;
		if (dladdr(callstack[i], &info) && info.dli_sname)
		{
			char* demangled = nullptr;
			int status = -1;
			if (info.dli_sname[0] == '_')
			{
				demangled = abi::__cxa_demangle(
					info.dli_sname, nullptr, nullptr, &status);
			}
			cse.symbolNameOriginal =
				info.dli_sname == nullptr ? symbols[i] : info.dli_sname;
			cse.symbolName =
				status == 0 ? std::string(demangled) : cse.symbolNameOriginal;
			free(demangled);
		}
		out_bt.backtrace_levels.emplace_back(cse);
	}
	free(symbols);
#endif
}

mrpt::system::TCallStackBackTrace::TCallStackBackTrace() = default;
std::string mrpt::system::TCallStackBackTrace::asString() const
{
	std::ostringstream trace_buf;
	trace_buf << "Callstack backtrace:" << std::endl;
	for (std::size_t i = 0; i < this->backtrace_levels.size(); i++)
	{
		trace_buf << mrpt::format(
						 "[%-2d] %*p %s", static_cast<int>(i),
						 int(2 + sizeof(void*) * 2),
						 backtrace_levels[i].address,
						 backtrace_levels[i].symbolName.c_str())
				  << std::endl;
	}
	return trace_buf.str();
}
