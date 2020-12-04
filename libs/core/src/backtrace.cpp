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
#include <mrpt/core/backtrace.h>
#include <mrpt/core/demangle.h>
#include <mrpt/core/format.h>

#include <iostream>
#include <sstream>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
//
#include <DbgHelp.h>
#else
#include <bfd.h>  // in deb package: binutils-dev
#include <dlfcn.h>  // dladdr()
#include <dlfcn.h>
#include <execinfo.h>
#include <link.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#endif

#ifndef _WIN32

// Partially based on code from:
// https://webcache.googleusercontent.com/search?q=cache:MXn9tpmIK5QJ:https://oroboro.com/printing-stack-traces-file-line/+&cd=2&hl=es&ct=clnk&gl=es

using u32 = uint32_t;
using s32 = int32_t;

class FileMatch
{
   public:
	FileMatch(void* addr) : mAddress(addr), mFile(NULL), mBase(NULL) {}

	void* mAddress;
	const char* mFile;
	void* mBase;
};
static int findMatchingFile(struct dl_phdr_info* info, size_t size, void* data)
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

static asymbol** kstSlurpSymtab(bfd* abfd, const char* fileName)
{
	if (!(bfd_get_file_flags(abfd) & HAS_SYMS))
	{
		printf(
			"Error bfd file \"%s\" flagged as having no symbols.\n", fileName);
		return NULL;
	}

	asymbol** syms;
	unsigned int size;

	long symcount = bfd_read_minisymbols(abfd, false, (void**)&syms, &size);
	if (symcount == 0)
		symcount = bfd_read_minisymbols(abfd, true, (void**)&syms, &size);

	if (symcount < 0)
	{
		printf("Error bfd file \"%s\", found no symbols.\n", fileName);
		return NULL;
	}

	return syms;
}

class FileLineDesc
{
   public:
	FileLineDesc(asymbol** syms, bfd_vma pc)
		: mPc(pc), mFound(false), mSyms(syms)
	{
	}

	void findAddressInSection(bfd* abfd, asection* section);

	bfd_vma mPc;
	char* mFilename;
	char* mFunctionname;
	unsigned int mLine;
	int mFound;
	asymbol** mSyms;
};

MRPT_TODO("Move to config.h!");

#define HAVE_DECL_BFD_GET_SECTION_FLAGS 1
#define HAVE_DECL_BFD_SECTION_VMA 1
#define HAVE_DECL_BFD_GET_SECTION_VMA 1
#define HAVE_1_ARG_BFD_SECTION_SIZE 0

#if HAVE_DECL_BFD_GET_SECTION_FLAGS
#define ucs_debug_bfd_section_flags(_abfd, _section) \
	bfd_get_section_flags(_abfd, _section)
#elif HAVE_DECL_BFD_SECTION_FLAGS
#define ucs_debug_bfd_section_flags(_abfd, _section) bfd_section_flags(_section)
#else
#error "Unsupported BFD API"
#endif

#if HAVE_DECL_BFD_GET_SECTION_VMA
#define ucs_debug_bfd_section_vma(_abfd, _section) \
	bfd_get_section_vma(_abfd, _section)
#elif HAVE_DECL_BFD_SECTION_VMA
#define ucs_debug_bfd_section_vma(_abfd, _section) bfd_section_vma(_section)
#else
#error "Unsupported BFD API"
#endif

#if HAVE_1_ARG_BFD_SECTION_SIZE
#define ucs_debug_bfd_section_size(_abfd, _section) bfd_section_size(_section)
#else
#define ucs_debug_bfd_section_size(_abfd, _section) \
	bfd_section_size(_abfd, _section);
#endif

void FileLineDesc::findAddressInSection(bfd* abfd, asection* section)
{
	if (mFound) return;

	if ((ucs_debug_bfd_section_flags(abfd, section) & SEC_ALLOC) == 0) return;

	bfd_vma vma = ucs_debug_bfd_section_vma(abfd, section);
	if (mPc < vma) return;

	bfd_size_type size = ucs_debug_bfd_section_size(abfd, section);
	if (mPc >= (vma + size)) return;

	mFound = bfd_find_nearest_line(
		abfd, section, mSyms, (mPc - vma), (const char**)&mFilename,
		(const char**)&mFunctionname, &mLine);
}

static void findAddressInSection(bfd* abfd, asection* section, void* data)
{
	FileLineDesc* desc = (FileLineDesc*)data;
	if (!desc) throw std::runtime_error("Cannot find debug symbol!");
	return desc->findAddressInSection(abfd, section);
}

std::vector<mrpt::TCallStackEntry> translateAddressesBuf(
	bfd* abfd, bfd_vma* addr, int numAddr, asymbol** syms)
{
	std::vector<mrpt::TCallStackEntry> ret;
	ret.resize(numAddr);

	for (int i = 0; i < numAddr; i++)
	{
		auto& e = ret.at(i);
		e.address = reinterpret_cast<void*>(addr[i]);

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

			if (desc.mFilename != NULL)
			{
				char* h = strrchr(desc.mFilename, '/');
				if (h != NULL) e.sourceFileName = h + 1;
			}
			e.sourceFileNumber = desc.mLine;
		}
	}

	return ret;
}

static std::vector<mrpt::TCallStackEntry> processFile(
	const char* fileName, bfd_vma* addr, int naddr)
{
	bfd* abfd = bfd_openr(fileName, NULL);
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

	const auto retBuf = translateAddressesBuf(abfd, addr, naddr, syms);

	free(syms);

	bfd_close(abfd);
	return retBuf;
}

std::vector<mrpt::TCallStackEntry> backtraceSymbols(
	void* const* addrList, int numAddr)
{
	//	char*** locations = (char***)alloca(sizeof(char**) * numAddr);

	// initialize the bfd library
	bfd_init();

	std::vector<mrpt::TCallStackEntry> ret;
	for (int idx = 0; idx < numAddr; idx++)
	{
		// find which executable, or library the symbol is from
		FileMatch match(addrList[idx]);
		dl_iterate_phdr(findMatchingFile, &match);

		// adjust the address in the global space of your binary to an
		// offset in the relevant library
		bfd_vma addr = (bfd_vma)(addrList[idx]);
		addr -= (bfd_vma)(match.mBase);

		// lookup the symbol
		const char* fil = (match.mFile && strlen(match.mFile))
							  ? match.mFile
							  : "/proc/self/exe";

		auto newVals = processFile(fil, &addr, 1);
		for (auto& v : newVals) ret.emplace_back(std::move(v));
	}

	return ret;
}
#endif

void mrpt::callStackBackTrace(TCallStackBackTrace& out_bt) noexcept
{
	out_bt.backtrace_levels.clear();
	// skip *this* function from the backtrace
	const unsigned int framesToSkip = 1;
	const unsigned int framesToCapture = 64;

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

	std::vector<void*> addrs;
	for (int i = (int)framesToSkip; i < nFrames; i++)
	{
		addrs.push_back(callstack[i]);
	}
	const auto ret = backtraceSymbols(addrs.data(), addrs.size());

	out_bt.backtrace_levels = ret;
	for (size_t i = 0; i < ret.size(); i++)
		out_bt.backtrace_levels[i].address = addrs[i];

	free(symbols);
#endif
}

mrpt::TCallStackBackTrace::TCallStackBackTrace() = default;
std::string mrpt::TCallStackBackTrace::asString() const noexcept
{
	std::ostringstream trace_buf;
	for (std::size_t i = 0; i < this->backtrace_levels.size(); i++)
	{
		trace_buf << mrpt::format(
						 "[%-2d] %*p %s", static_cast<int>(i),
						 int(2 + sizeof(void*) * 2),
						 backtrace_levels[i].address,
						 backtrace_levels[i].symbolName.c_str())
				  << "\n";
	}
	return trace_buf.str();
}
