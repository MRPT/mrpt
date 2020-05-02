
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

#ifndef JOURNALEXCEPTION_H
#define JOURNALEXCEPTION_H

#include <string>
#include <xstypes/xsexception.h>

#include "journaller.h"

#ifndef XSENS_WINDOWS
#include <signal.h>
#include <cstring>
#endif

class JournalException : public XsException {
public:
	JournalException(std::string const& message);
	~JournalException() throw() {}

	const char *msg() const;
	const std::string& stack() const;

protected:
	std::string m_stack; //!< A string that contains a stack dump
	friend class JournalExceptionStackWalker;
};

//! Use this macro to throw an exception that includes callstack information
#define JLTHROW(msg)	do { std::ostringstream os; os << msg; throw JournalException(os.str()); } while(0)

extern bool gOnExceptionGotoDebugger;

#ifdef XSENS_WINDOWS
	#include <windows.h>
	LONG journallerExceptionFilter(EXCEPTION_POINTERS* pExPtrs, Journaller*);
#endif

#if JLDEF_BUILD > JLL_FATAL
	#define JOURNALCRASHES_SIGNAL_FUNCTIONS
	#define JOURNALCRASHES_BEGIN(journal)
	#define JOURNALCRASHES_END(journal)
#else
	#ifdef XSENS_WINDOWS
		#define JOURNALCRASHES_SIGNAL_FUNCTIONS
		#define JOURNALCRASHES_BEGIN(journal) \
			__try {

		#define JOURNALCRASHES_END(journal) \
			} __except (journallerExceptionFilter(GetExceptionInformation(), journal)) {}

	#else
		#define JOURNALCRASHES_SIGNAL_FUNCTIONS \
			Journaller *gSCJ = 0;\
			extern "C" void signal_handler(int signal, siginfo_t *, void *)\
			{\
				if (gSCJ)\
				{\
					JLFATAL_NODEC(gSCJ, strsignal(signal));\
					gSCJ->writeCallstack(JLL_Fatal);\
				}\
				_exit(-1);\
			}

		#define JOURNALCRASHES_BEGIN(journal) \
			do { \
				gSCJ = journal;\
				struct sigaction act;\
				memset(&act, 0, sizeof(act));\
				act.sa_flags = SA_SIGINFO;\
				act.sa_sigaction = &signal_handler;\
				sigaction(SIGSEGV, &act, NULL);\
				sigaction(SIGILL, &act, NULL);\
				sigaction(SIGABRT, &act, NULL);\
				sigaction(SIGFPE, &act, NULL);\
			} while (0);
		#define JOURNALCRASHES_END(journal)
	#endif
#endif

#if JLDEF_BUILD > JLL_ERROR
	#define JOURNALEXCEPTIONS_BEGIN(journal)		try {
	#define JOURNALEXCEPTIONS_END_NOTHROW(journal)	} catch(...) { }
	#define JOURNALEXCEPTIONS_END_RETHROW(journal)	} catch(...) { throw; }
#else
	#define JOURNALEXCEPTIONS_BEGIN(journal) \
		try {

	#define JOURNALEXCEPTIONS_END_NOTHROW(journal) \
		} catch (JournalException& e) { JLERROR(journal, e.msg()); JLERROR_NODEC(journal, e.stack()); }\
		catch (XsException& e) { JLERROR(journal, e.what()); }\
		catch (std::exception& e) { JLERROR(journal, e.what()); }

	#define JOURNALEXCEPTIONS_END_RETHROW(journal) \
		} catch (JournalException& e) { JLERROR(journal, e.msg()); JLERROR_NODEC(journal, e.stack()); throw; }\
		catch (XsException& e) { JLERROR(journal, e.what()); throw; }\
		catch (std::exception& e) { JLERROR(journal, e.what()); throw; }
#endif

#endif
