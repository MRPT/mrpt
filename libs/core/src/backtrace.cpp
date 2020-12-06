/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "core-precomp.h"  // Precompiled headers
//
#include <mrpt/config.h>  // MRPT_HAS_BFD, etc.
#include <mrpt/core/backtrace.h>
#include <mrpt/core/demangle.h>
#include <mrpt/core/format.h>

#include <iostream>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
//
#include <DbgHelp.h>
#else
#include <dlfcn.h>  // dladdr()
#include <dlfcn.h>
#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#endif

#if MRPT_HAS_BFD
// Partially based on code from:
// https://webcache.googleusercontent.com/search?q=cache:MXn9tpmIK5QJ:https://oroboro.com/printing-stack-traces-file-line/+&cd=2&hl=es&ct=clnk&gl=es

#include <bfd.h>  // in deb package: binutils-dev
#include <link.h> // in deb package: libc6-dev

#if HAVE_DECL_BFD_GET_SECTION_FLAGS
#define mrpt_debug_bfd_section_flags(_abfd, _section) \
	bfd_get_section_flags(_abfd, _section)
#elif HAVE_DECL_BFD_SECTION_FLAGS
#define mrpt_debug_bfd_section_flags(_abfd, _section) \
	bfd_section_flags(_section)
#else
#error "Unsupported BFD API"
#endif

#if HAVE_DECL_BFD_GET_SECTION_VMA
#define mrpt_debug_bfd_section_vma(_abfd, _section) \
	bfd_get_section_vma(_abfd, _section)
#elif HAVE_DECL_BFD_SECTION_VMA
#define mrpt_debug_bfd_section_vma(_abfd, _section) bfd_section_vma(_section)
#else
#error "Unsupported BFD API"
#endif

#if HAVE_1_ARG_BFD_SECTION_SIZE
#define mrpt_debug_bfd_section_size(_abfd, _section) bfd_section_size(_section)
#else
#define mrpt_debug_bfd_section_size(_abfd, _section) \
	bfd_section_size(_abfd, _section);
#endif

using u32 = uint32_t;
using s32 = int32_t;

class FileMatch
{
   public:
	FileMatch(void* addr) noexcept : mAddress(addr) {}

	void* mAddress;
	const char* mFile = nullptr;
	void* mBase = nullptr;
};
static int findMatchingFile(
	struct dl_phdr_info* info, size_t /*size*/, void* data) noexcept
{
	FileMatch* match = (FileMatch*)data;

	for (u32 i = 0; i < info->dlpi_phnum; i++)
	{
		const ElfW(Phdr)& phdr = info->dlpi_phdr[i];

		if (phdr.p_type == PT_LOAD)
		{
			ElfW(Addr) vaddr = phdr.p_vaddr + info->dlpi_addr;
			ElfW(Addr) maddr = ElfW(Addr)(match->mAddress);
			if ((maddr >= vaddr) && (maddr < vaddr + phdr.p_memsz))
			{
				match->mFile = info->dlpi_name;
				match->mBase = (void*)info->dlpi_addr;
				return 1;
			}
		}
	}
	return 0;
}

static asymbol** kstSlurpSymtab(bfd* abfd, const char* fileName) noexcept
{
	if (!(bfd_get_file_flags(abfd) & HAS_SYMS))
	{
		printf(
			"Error bfd file \"%s\" flagged as having no symbols.\n", fileName);
		return nullptr;
	}

	asymbol** syms;
	unsigned int size;

	long symcount = bfd_read_minisymbols(abfd, false, (void**)&syms, &size);
	if (symcount == 0)
		symcount = bfd_read_minisymbols(abfd, true, (void**)&syms, &size);

	if (symcount < 0)
	{
		printf("Error bfd file \"%s\", found no symbols.\n", fileName);
		return nullptr;
	}

	return syms;
}

class FileLineDesc
{
   public:
	FileLineDesc(asymbol** syms, bfd_vma pc) noexcept : mPc(pc), mSyms(syms) {}

	void findAddressInSection(bfd* abfd, asection* section) noexcept;

	bfd_vma mPc;
	char* mFilename;
	char* mFunctionname;
	unsigned int mLine;
	bool mFound = false;
	asymbol** mSyms;
};

void FileLineDesc::findAddressInSection(bfd* abfd, asection* section) noexcept
{
	if (mFound) return;

	if ((mrpt_debug_bfd_section_flags(abfd, section) & SEC_ALLOC) == 0) return;

	bfd_vma vma = mrpt_debug_bfd_section_vma(abfd, section);
	if (mPc < vma) return;

	bfd_size_type size = mrpt_debug_bfd_section_size(abfd, section);
	if (mPc >= (vma + size)) return;

	mFound = bfd_find_nearest_line(
		abfd, section, mSyms, (mPc - vma), (const char**)&mFilename,
		(const char**)&mFunctionname, &mLine);
}

static void findAddressInSection(
	bfd* abfd, asection* section, void* data) noexcept
{
	FileLineDesc* desc = (FileLineDesc*)data;
	if (!desc)
	{
		std::cerr << "[mrpt::callStackBackTrace()] ERROR: Cannot find debug "
					 "symbol.\n";
		return;
	}
	return desc->findAddressInSection(abfd, section);
}

std::vector<mrpt::TCallStackEntry> translateAddressesBuf(
	bfd* abfd, bfd_vma* addr, int numAddr, asymbol** syms) noexcept
{
	std::vector<mrpt::TCallStackEntry> ret;
	ret.resize(numAddr);

	for (int i = 0; i < numAddr; i++)
	{
		auto& e = ret.at(i);

		FileLineDesc desc(syms, addr[i]);

		bfd_map_over_sections(abfd, &findAddressInSection, (void*)&desc);

		if (!desc.mFound)
		{
			e.sourceFileName.clear();  // empty=no symbol name available
		}
		else
		{
			e.symbolNameOriginal = desc.mFunctionname;

			if (!e.symbolNameOriginal.empty())
				e.symbolName = mrpt::demangle(e.symbolNameOriginal);

			if (desc.mFilename != nullptr)
			{
				char* h = strrchr(desc.mFilename, '/');
				if (h != nullptr) e.sourceFileName = h + 1;
				e.sourceFileFullPath = desc.mFilename;
			}
			e.sourceFileNumber = desc.mLine;
		}
	}

	return ret;
}

static std::vector<mrpt::TCallStackEntry> processFile(
	const char* fileName, bfd_vma* addr, int naddr,
	void* const PC_addr) noexcept
{
	bfd* abfd = bfd_openr(fileName, nullptr);
	if (!abfd)
	{
		printf("Error opening bfd file \"%s\"\n", fileName);
		return {};
	}

	if (bfd_check_format(abfd, bfd_archive))
	{
		printf("Cannot get addresses from archive \"%s\"\n", fileName);
		bfd_close(abfd);
		return {};
	}

	char** matching;
	if (!bfd_check_format_matches(abfd, bfd_object, &matching))
	{
		printf("Format does not match for archive \"%s\"\n", fileName);
		bfd_close(abfd);
		return {};
	}

	asymbol** syms = kstSlurpSymtab(abfd, fileName);
	if (!syms)
	{
		printf("Failed to read symbol table for archive \"%s\"\n", fileName);
		bfd_close(abfd);
		return {};
	}

	auto retBuf = translateAddressesBuf(abfd, addr, naddr, syms);
	for (auto& e : retBuf) e.address = PC_addr;

	free(syms);

	bfd_close(abfd);
	return retBuf;
}

static std::vector<mrpt::TCallStackEntry> backtraceSymbols(
	void* const* addrList, int numAddr) noexcept
{
	// initialize the bfd library
	bfd_init();

	std::vector<mrpt::TCallStackEntry> ret;
	for (int idx = 0; idx < numAddr; idx++)
	{
		// find which executable, or library the symbol is from
		void* const PC_addr = addrList[idx];
		FileMatch match(PC_addr);
		dl_iterate_phdr(findMatchingFile, &match);

		// adjust the address in the global space of your binary to an
		// offset in the relevant library
		bfd_vma addr = (bfd_vma)(addrList[idx]);
		addr -= (bfd_vma)(match.mBase);

		// lookup the symbol
		const char* fil = (match.mFile && strlen(match.mFile))
							  ? match.mFile
							  : "/proc/self/exe";

		auto newVals = processFile(fil, &addr, 1, PC_addr);
		for (auto& v : newVals) ret.emplace_back(std::move(v));
	}

	return ret;
}
#endif

void mrpt::callStackBackTrace(
	TCallStackBackTrace& out_bt, const unsigned int framesToSkip,
	const unsigned int framesToCapture) noexcept
{
	out_bt.backtrace_levels.clear();

#ifdef _WIN32
	void* backTrace[framesToCapture]{};

	SymSetOptions(SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
	const HANDLE hProcess = GetCurrentProcess();
	if (!SymInitialize(
			hProcess, nullptr /* UserSearchPath  */, TRUE /*fInvadeProcess*/))
	{
		std::cerr << "[mrpt::callStackBackTrace] Error in SymInitialize()!"
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
		cse.symbolName = mrpt::demangle(si.Name);

		out_bt.backtrace_levels.emplace_back(cse);
	}
#else
	// Based on: https://gist.github.com/fmela/591333
	void* callstack[framesToCapture];
	const int nMaxFrames = sizeof(callstack) / sizeof(callstack[0]);
	int nFrames = ::backtrace(callstack, nMaxFrames);
	char** symbols = ::backtrace_symbols(callstack, nFrames);

#if MRPT_HAS_BFD
	// Use BFD to solve for symbol names and line numbers.
	std::vector<void*> addrs;
	for (int i = (int)framesToSkip; i < nFrames; i++)
		addrs.push_back(callstack[i]);

	out_bt.backtrace_levels = backtraceSymbols(addrs.data(), addrs.size());

#else
	// If BFD is not available, solve for symbol names only:
	for (int i = (int)framesToSkip; i < nFrames; i++)
	{
		TCallStackEntry cse;
		cse.address = callstack[i];

		Dl_info info;
		if (dladdr(callstack[i], &info) && info.dli_sname)
		{
			cse.symbolNameOriginal =
				info.dli_sname == nullptr ? symbols[i] : info.dli_sname;
			cse.symbolName = mrpt::demangle(cse.symbolNameOriginal);
		}
		out_bt.backtrace_levels.emplace_back(cse);
	}
#endif

	free(symbols);
#endif
}

mrpt::TCallStackBackTrace::TCallStackBackTrace() = default;
std::string mrpt::TCallStackBackTrace::asString() const noexcept
{
	std::string trace_buf;
	for (std::size_t i = 0; i < backtrace_levels.size(); i++)
	{
		const auto& bt = backtrace_levels[i];

		trace_buf += mrpt::format("[%-2d] ", static_cast<int>(i));

		if (bt.symbolName.empty())
		{
			trace_buf +=
				mrpt::format("%*p", int(2 + sizeof(void*) * 2), bt.address);
		}
		else
		{
			if (!bt.sourceFileName.empty())
			{
				trace_buf += bt.sourceFileFullPath;
				trace_buf += ":";
				trace_buf += std::to_string(bt.sourceFileNumber);
				trace_buf += " ";
			}
			else
			{
				trace_buf += "(unknown file) ";
			}

			trace_buf += bt.symbolName;
		}
		trace_buf += "\n";
	}
	return trace_buf;
}
